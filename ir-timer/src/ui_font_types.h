#pragma once
#include <stdint.h>

// tools/gen_fonts.ps1 が作る 1 ビットのフォント(src/ui_fonts.h)の型
struct UiGlyph {
  uint32_t code;   // Unicode
  int16_t adv;     // 送り幅
  int16_t x, y;    // 描画原点(行の左上)からのインクの位置
  uint16_t w, h;   // インクの大きさ。ビットマップは MSB 先頭、行ごとにバイト境界
  uint32_t off;    // bits 内の位置
};

struct UiFont {
  const UiGlyph* glyphs;  // code の昇順
  uint16_t count;
  const uint8_t* bits;
  int16_t px;
};
