#include <Arduino.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include "LGFX_Elecrow_5inch.hpp"

namespace {
constexpr uint32_t kUsbBaudRate = 115200;
constexpr uint32_t kTargetBaudRate = 115200;
constexpr int kTargetRxPin = 44;
constexpr int kTargetTxPin = 43;
constexpr bool kLocalEcho = true;
constexpr uint8_t kCellWidth = 12;
constexpr uint8_t kCellHeight = 16;
constexpr uint8_t kTabWidth = 8;
constexpr size_t kMaxCsiBufferSize = 32;
constexpr uint32_t kCursorBlinkMs = 500;
constexpr uint32_t kLoopDelayMs = 2;
constexpr uint16_t kDefaultForeground = TFT_GREEN;
constexpr uint16_t kDefaultBackground = TFT_BLACK;
}  // namespace

class TerminalView {
 public:
  struct Cell {
    char ch = ' ';
    uint16_t fg = kDefaultForeground;
    uint16_t bg = kDefaultBackground;
  };

  void begin(LGFX& display) {
    display_ = &display;
    display_->init();
    display_->setRotation(0);
    display_->setBrightness(192);
    display_->setColorDepth(16);
    display_->setFont(&fonts::Font0);
    display_->setTextSize(2);
    display_->setTextWrap(false, false);
    display_->fillScreen(kDefaultBackground);

    cols_ = std::max<uint16_t>(1, display_->width() / kCellWidth);
    rows_ = std::max<uint16_t>(1, display_->height() / kCellHeight);
    cells_.assign(static_cast<size_t>(cols_) * rows_, Cell{});
    dirty_rows_.assign(rows_, true);

    cursor_col_ = 0;
    cursor_row_ = 0;
    saved_col_ = 0;
    saved_row_ = 0;
    resetAttributes();
    cursor_visible_ = true;
    blink_state_ = true;
    last_blink_ms_ = millis();

    writeString("Elecrow 5\" LovyanGFX VT100 serial monitor\r\n");
    writeString("USB CDC <-> UART1 bridge ready.\r\n");
    writeString("UART1 RX=GPIO44 TX=GPIO43 115200bps\r\n\r\n");
    render(true);
  }

  void feed(uint8_t byte) {
    switch (state_) {
      case ParseState::kGround:
        handleGround(byte);
        break;
      case ParseState::kEscape:
        handleEscape(byte);
        break;
      case ParseState::kCsi:
        handleCsi(byte);
        break;
    }
  }

  void render(bool force = false) {
    if (millis() - last_blink_ms_ >= kCursorBlinkMs) {
      blink_state_ = !blink_state_;
      last_blink_ms_ = millis();
      markCellDirty(cursor_row_);
    }

    if (force) {
      std::fill(dirty_rows_.begin(), dirty_rows_.end(), true);
    }

    for (uint16_t row = 0; row < rows_; ++row) {
      if (!dirty_rows_[row]) {
        continue;
      }
      for (uint16_t col = 0; col < cols_; ++col) {
        drawCell(col, row, cursor_visible_ && blink_state_ && row == cursor_row_ && col == cursor_col_);
      }
      dirty_rows_[row] = false;
    }
  }

  void writeString(const char* text) {
    while (*text != '\0') {
      feed(static_cast<uint8_t>(*text++));
    }
  }

 private:
  enum class ParseState { kGround, kEscape, kCsi };

  LGFX* display_ = nullptr;
  std::vector<Cell> cells_;
  std::vector<bool> dirty_rows_;
  uint16_t cols_ = 0;
  uint16_t rows_ = 0;
  uint16_t cursor_col_ = 0;
  uint16_t cursor_row_ = 0;
  uint16_t saved_col_ = 0;
  uint16_t saved_row_ = 0;
  uint16_t current_fg_ = kDefaultForeground;
  uint16_t current_bg_ = kDefaultBackground;
  bool bold_ = false;
  int fg_index_ = -1;
  int bg_index_ = -1;
  bool bg_bright_ = false;
  bool inverse_ = false;
  bool cursor_visible_ = true;
  bool blink_state_ = true;
  uint32_t last_blink_ms_ = 0;
  ParseState state_ = ParseState::kGround;
  std::string csi_buffer_;

  Cell& cell(uint16_t col, uint16_t row) {
    return cells_[static_cast<size_t>(row) * cols_ + col];
  }

  void handleGround(uint8_t byte) {
    switch (byte) {
      case 0x1B:
        state_ = ParseState::kEscape;
        return;
      case '\r':
        setCursor(cursor_row_, 0);
        return;
      case '\n':
        newLine();
        return;
      case '\b':
        if (cursor_col_ > 0) {
          setCursor(cursor_row_, cursor_col_ - 1);
        }
        return;
      case '\t': {
        uint16_t next_tab = static_cast<uint16_t>(((cursor_col_ / kTabWidth) + 1U) * kTabWidth);
        while (cursor_col_ < cols_ && cursor_col_ < next_tab) {
          putCharacter(' ');
        }
        return;
      }
      default:
        break;
    }

    if (byte >= 0x20 && byte <= 0x7E) {
      putCharacter(static_cast<char>(byte));
    }
  }

  void handleEscape(uint8_t byte) {
    switch (byte) {
      case '[':
        csi_buffer_.clear();
        state_ = ParseState::kCsi;
        return;
      case '7':
        saved_col_ = cursor_col_;
        saved_row_ = cursor_row_;
        state_ = ParseState::kGround;
        return;
      case '8':
        setCursor(saved_row_, saved_col_);
        state_ = ParseState::kGround;
        return;
      case 'c':
        resetTerminal();
        state_ = ParseState::kGround;
        return;
      default:
        state_ = ParseState::kGround;
        return;
    }
  }

  void handleCsi(uint8_t byte) {
    if ((byte >= '0' && byte <= '9') || byte == ';' || byte == '?') {
      if (csi_buffer_.size() < kMaxCsiBufferSize) {
        csi_buffer_.push_back(static_cast<char>(byte));
      }
      return;
    }

    executeCsi(static_cast<char>(byte));
    state_ = ParseState::kGround;
  }

  void executeCsi(char command) {
    bool private_mode = !csi_buffer_.empty() && csi_buffer_.front() == '?';
    std::string payload = private_mode ? csi_buffer_.substr(1) : csi_buffer_;
    std::vector<int> params = parseParameters(payload);

    if (private_mode && (command == 'h' || command == 'l')) {
      if (!params.empty() && params.front() == 25) {
        cursor_visible_ = command == 'h';
        markCellDirty(cursor_row_);
      }
      return;
    }

    switch (command) {
      case 'A':
        moveCursorRelative(-static_cast<int>(clampCount(params, 1)), 0);
        break;
      case 'B':
        moveCursorRelative(static_cast<int>(clampCount(params, 1)), 0);
        break;
      case 'C':
        moveCursorRelative(0, static_cast<int>(clampCount(params, 1)));
        break;
      case 'D':
        moveCursorRelative(0, -static_cast<int>(clampCount(params, 1)));
        break;
      case 'H':
      case 'f': {
        int target_row = params.size() > 0 ? std::max(params[0], 1) - 1 : 0;
        int target_col = params.size() > 1 ? std::max(params[1], 1) - 1 : 0;
        setCursor(static_cast<uint16_t>(target_row), static_cast<uint16_t>(target_col));
        break;
      }
      case 'J':
        eraseInDisplay(params.empty() ? 0 : params[0]);
        break;
      case 'K':
        eraseInLine(params.empty() ? 0 : params[0]);
        break;
      case 'm':
        applySelectGraphicRendition(params);
        break;
      case 's':
        saved_col_ = cursor_col_;
        saved_row_ = cursor_row_;
        break;
      case 'u':
        setCursor(saved_row_, saved_col_);
        break;
      default:
        break;
    }
  }

  std::vector<int> parseParameters(const std::string& payload) const {
    if (payload.empty()) {
      return {};
    }

    std::vector<int> params;
    size_t start = 0;
    while (start <= payload.size()) {
      size_t end = payload.find(';', start);
      std::string token = payload.substr(start, end == std::string::npos ? std::string::npos : end - start);
      params.push_back(token.empty() ? 0 : std::atoi(token.c_str()));
      if (end == std::string::npos) {
        break;
      }
      start = end + 1;
    }
    return params;
  }

  uint16_t clampCount(const std::vector<int>& params, uint16_t fallback) const {
    int count = params.empty() || params.front() <= 0 ? fallback : params.front();
    return static_cast<uint16_t>(std::max(0, count));
  }

  void applySelectGraphicRendition(const std::vector<int>& params) {
    if (params.empty()) {
      resetAttributes();
      return;
    }

    for (int code : params) {
      switch (code) {
        case 0:
          resetAttributes();
          break;
        case 1:
          bold_ = true;
          if (fg_index_ >= 0) {
            current_fg_ = mapAnsiColor(fg_index_, true, false);
          }
          break;
        case 7:
          inverse_ = true;
          break;
        case 22:
          bold_ = false;
          if (fg_index_ >= 0) {
            current_fg_ = mapAnsiColor(fg_index_, false, false);
          } else {
            current_fg_ = kDefaultForeground;
          }
          break;
        case 27:
          inverse_ = false;
          break;
        case 39:
          fg_index_ = -1;
          current_fg_ = kDefaultForeground;
          break;
        case 49:
          bg_index_ = -1;
          bg_bright_ = false;
          current_bg_ = kDefaultBackground;
          break;
        default:
          if (code >= 30 && code <= 37) {
            fg_index_ = code - 30;
            current_fg_ = mapAnsiColor(fg_index_, bold_, false);
          } else if (code >= 40 && code <= 47) {
            bg_index_ = code - 40;
            bg_bright_ = false;
            current_bg_ = mapAnsiColor(bg_index_, false, true);
          } else if (code >= 90 && code <= 97) {
            fg_index_ = code - 90;
            current_fg_ = mapAnsiColor(fg_index_, true, false);
          } else if (code >= 100 && code <= 107) {
            bg_index_ = code - 100;
            bg_bright_ = true;
            current_bg_ = mapAnsiColor(bg_index_, true, true);
          }
          break;
      }
    }
  }

  uint16_t mapAnsiColor(int index, bool bright, bool background) const {
    static constexpr uint16_t colors[8] = {
        TFT_BLACK, TFT_RED, TFT_GREEN, TFT_YELLOW,
        TFT_BLUE, TFT_MAGENTA, TFT_CYAN, TFT_WHITE,
    };
    static constexpr uint16_t bright_colors[8] = {
        0x7BEF,   // bright black
        0xFBEA,   // bright red
        0x87F0,   // bright green
        0xFFF0,   // bright yellow
        0x7D7C,   // bright blue
        0xFD5F,   // bright magenta
        0x87FF,   // bright cyan
        TFT_WHITE // bright white
    };

    uint16_t color = bright ? bright_colors[index % 8] : colors[index % 8];
    if (background && color == TFT_BLACK) {
      return bright ? 0x4208 : TFT_BLACK;
    }
    return color;
  }

  void resetAttributes() {
    bold_ = false;
    fg_index_ = -1;
    bg_index_ = -1;
    bg_bright_ = false;
    inverse_ = false;
    current_fg_ = kDefaultForeground;
    current_bg_ = kDefaultBackground;
  }

  void eraseInDisplay(int mode) {
    switch (mode) {
      case 2:
        clearAll();
        setCursor(0, 0);
        break;
      case 1:
        for (uint16_t row = 0; row <= cursor_row_; ++row) {
          uint16_t limit = row == cursor_row_ ? cursor_col_ + 1 : cols_;
          for (uint16_t col = 0; col < limit; ++col) {
            clearCell(col, row);
          }
        }
        break;
      case 0:
      default:
        for (uint16_t row = cursor_row_; row < rows_; ++row) {
          uint16_t begin = row == cursor_row_ ? cursor_col_ : 0;
          for (uint16_t col = begin; col < cols_; ++col) {
            clearCell(col, row);
          }
        }
        break;
    }
  }

  void eraseInLine(int mode) {
    switch (mode) {
      case 2:
        for (uint16_t col = 0; col < cols_; ++col) {
          clearCell(col, cursor_row_);
        }
        break;
      case 1:
        for (uint16_t col = 0; col <= cursor_col_; ++col) {
          clearCell(col, cursor_row_);
        }
        break;
      case 0:
      default:
        for (uint16_t col = cursor_col_; col < cols_; ++col) {
          clearCell(col, cursor_row_);
        }
        break;
    }
  }

  void clearAll() {
    for (uint16_t row = 0; row < rows_; ++row) {
      for (uint16_t col = 0; col < cols_; ++col) {
        clearCell(col, row);
      }
    }
  }

  void clearCell(uint16_t col, uint16_t row) {
    Cell& target = cell(col, row);
    target = Cell{};
    markCellDirty(row);
  }

  void putCharacter(char ch) {
    Cell& target = cell(cursor_col_, cursor_row_);
    target.ch = ch;
    target.fg = inverse_ ? current_bg_ : current_fg_;
    target.bg = inverse_ ? current_fg_ : current_bg_;
    markCellDirty(cursor_row_);

    if (cursor_col_ + 1 >= cols_) {
      newLine();
    } else {
      setCursor(cursor_row_, cursor_col_ + 1);
    }
  }

  void setCursor(uint16_t row, uint16_t col) {
    uint16_t old_row = cursor_row_;
    cursor_row_ = std::min<uint16_t>(row, rows_ - 1);
    cursor_col_ = std::min<uint16_t>(col, cols_ - 1);
    markCellDirty(old_row);
    markCellDirty(cursor_row_);
  }

  void moveCursorRelative(int row_delta, int col_delta) {
    int row = static_cast<int>(cursor_row_) + row_delta;
    int col = static_cast<int>(cursor_col_) + col_delta;
    row = std::max(0, std::min(row, static_cast<int>(rows_ - 1)));
    col = std::max(0, std::min(col, static_cast<int>(cols_ - 1)));
    setCursor(static_cast<uint16_t>(row), static_cast<uint16_t>(col));
  }

  void newLine() {
    uint16_t old_row = cursor_row_;
    cursor_col_ = 0;
    if (cursor_row_ + 1 >= rows_) {
      scrollUp();
    } else {
      cursor_row_++;
    }
    markCellDirty(old_row);
    markCellDirty(cursor_row_);
  }

  void scrollUp() {
    for (uint16_t row = 1; row < rows_; ++row) {
      for (uint16_t col = 0; col < cols_; ++col) {
        cell(col, row - 1) = cell(col, row);
      }
      dirty_rows_[row - 1] = true;
    }
    for (uint16_t col = 0; col < cols_; ++col) {
      cell(col, rows_ - 1) = Cell{};
    }
    dirty_rows_[rows_ - 1] = true;
  }

  void resetTerminal() {
    clearAll();
    resetAttributes();
    setCursor(0, 0);
  }

  void drawCell(uint16_t col, uint16_t row, bool show_cursor) {
    const Cell& source = cell(col, row);
    uint16_t fg = source.fg;
    uint16_t bg = source.bg;
    if (show_cursor) {
      std::swap(fg, bg);
    }

    int16_t x = static_cast<int16_t>(col * kCellWidth);
    int16_t y = static_cast<int16_t>(row * kCellHeight);
    display_->fillRect(x, y, kCellWidth, kCellHeight, bg);
    display_->setTextColor(fg, bg);
    display_->setCursor(x, y);
    display_->print(source.ch);
  }

  void markCellDirty(uint16_t row) {
    if (row < dirty_rows_.size()) {
      dirty_rows_[row] = true;
    }
  }
};

LGFX display;
TerminalView terminal_view;
HardwareSerial target_uart(1);

void setup() {
  Serial.begin(kUsbBaudRate);
  delay(200);

  terminal_view.begin(display);

  target_uart.begin(kTargetBaudRate, SERIAL_8N1, kTargetRxPin, kTargetTxPin);

  Serial.println();
  Serial.println("Elecrow VT100 monitor booted.");
}

void loop() {
  while (target_uart.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(target_uart.read());
    terminal_view.feed(byte);
    Serial.write(byte);
  }

  while (Serial.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    target_uart.write(byte);
    if (kLocalEcho) {
      terminal_view.feed(byte);
    }
  }

  terminal_view.render();
  delay(kLoopDelayMs);
}
