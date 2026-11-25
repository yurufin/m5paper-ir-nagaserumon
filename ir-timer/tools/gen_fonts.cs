// ir-timer の画面用フォントを Windows の TTF から 1 ビットのビットマップにして src/ui_fonts.h を作る
// gen_fonts.ps1 から Add-Type で読み込む。文字はここに並べたものだけ入るので、画面の文言を増やしたらここにも足す
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Drawing.Text;
using System.IO;
using System.Linq;
using System.Text;

public static class UiFontGen {
    const string ASCII = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";

    class Spec {
        public string Name; public int Px; public string Chars;
        public Spec(string n, int px, string c) { Name = n; Px = px; Chars = c; }
    }

    static readonly Spec[] Specs = {
        new Spec("FONT_STATUS", 26, "0123456789%-"),
        new Spec("FONT_LOGO",   34, "LIXIL"),
        new Spec("FONT_TITLE",  84, "流せるもん!"),
        new Spec("FONT_DATE",   46, "0123456789/ ()-日月火水木金土時刻未設定"),
        new Spec("FONT_TIME",  160, "0123456789:-"),
        new Spec("FONT_BUTTON", 62, "今すぐ流す送信中…"),
        new Spec("FONT_HINT",   23, ASCII + "★サイドの押込ボタンでも流せます"),
        new Spec("FONT_LABEL",  31, "◆自動洗浄時刻最終"),
        new Spec("FONT_SLOT",   40, "0123456789:"),
        new Spec("FONT_LAST",   33, ASCII + "手動自動なし時刻不明"),
        new Spec("FONT_BAR",    32, ASCII + "失敗信号ファイルがありません時刻未設定のため自動洗浄停止中読めない"),
    };

    class Glyph { public int Code, Adv, X, Y, W, H; public byte[] Bits; }

    public static void Run(string family, string outPath) {
        var sb = new StringBuilder();
        sb.Append("// 自動生成: tools/gen_fonts.ps1(フォント: " + family + " Bold)。手で直さない\n");
        sb.Append("#pragma once\n#include \"ui_font_types.h\"\n\n");
        foreach (var spec in Specs) {
            var codes = new SortedSet<int>();
            for (int i = 0; i < spec.Chars.Length; i++) codes.Add(char.ConvertToUtf32(spec.Chars, i));
            var glyphs = new List<Glyph>();
            int ascentTop = int.MaxValue;
            using (var font = new Font(family, spec.Px, FontStyle.Bold, GraphicsUnit.Pixel)) {
                foreach (int code in codes) glyphs.Add(Render(font, spec.Px, code));
            }
            var bits = new List<byte>();
            string lname = spec.Name.ToLower();
            sb.AppendFormat("static const uint8_t {0}_bits[] = {{", lname);
            var offs = new List<int>();
            foreach (var g in glyphs) { offs.Add(bits.Count); bits.AddRange(g.Bits); }
            for (int i = 0; i < bits.Count; i++) {
                if (i % 24 == 0) sb.Append("\n  ");
                sb.AppendFormat("0x{0:x2},", bits[i]);
            }
            sb.Append("\n};\n");
            sb.AppendFormat("static const UiGlyph {0}_glyphs[] = {{\n", lname);
            for (int i = 0; i < glyphs.Count; i++) {
                var g = glyphs[i];
                sb.AppendFormat("  {{0x{0:x}, {1}, {2}, {3}, {4}, {5}, {6}}},\n", g.Code, g.Adv, g.X, g.Y, g.W, g.H, offs[i]);
                if (g.H > 0) ascentTop = Math.Min(ascentTop, g.Y);
            }
            sb.Append("};\n");
            sb.AppendFormat("static const UiFont {0} = {{{1}_glyphs, {2}, {1}_bits, {3}}};\n\n", spec.Name, lname, glyphs.Count, spec.Px);
        }
        File.WriteAllText(outPath, sb.ToString().Replace("\r\n", "\n"), new UTF8Encoding(false));
    }

    // 1 文字を大きめのキャンバスに描き、インクのある範囲だけを 1 ビット(MSB 先頭、行ごとにバイト境界)で取る
    // X, Y は「文字の描画原点(行の左上)」からのインクの位置
    static Glyph Render(Font font, int px, int code) {
        string s = char.ConvertFromUtf32(code);
        int size = px * 3;
        int ox = px, oy = px / 2;
        var g = new Glyph { Code = code };
        var fmt = StringFormat.GenericTypographic;
        fmt.FormatFlags |= StringFormatFlags.MeasureTrailingSpaces;
        using (var bmp = new Bitmap(size, size, PixelFormat.Format32bppArgb))
        using (var gr = Graphics.FromImage(bmp)) {
            gr.Clear(Color.White);
            gr.TextRenderingHint = TextRenderingHint.AntiAliasGridFit;
            gr.DrawString(s, font, Brushes.Black, ox, oy, fmt);
            g.Adv = (int)Math.Round(gr.MeasureString(s, font, size, fmt).Width);
            if (code == ' ') g.Adv = (int)Math.Round(px * 0.3);
            int minX = size, minY = size, maxX = -1, maxY = -1;
            var dark = new bool[size, size];
            var data = bmp.LockBits(new Rectangle(0, 0, size, size), ImageLockMode.ReadOnly, PixelFormat.Format32bppArgb);
            var raw = new byte[data.Stride * size];
            System.Runtime.InteropServices.Marshal.Copy(data.Scan0, raw, 0, raw.Length);
            bmp.UnlockBits(data);
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++) {
                    int lum = raw[y * data.Stride + x * 4 + 1];  // G
                    if (lum < 128) {
                        dark[x, y] = true;
                        if (x < minX) minX = x; if (x > maxX) maxX = x;
                        if (y < minY) minY = y; if (y > maxY) maxY = y;
                    }
                }
            if (maxX < 0) { g.Bits = new byte[0]; return g; }
            g.X = minX - ox; g.Y = minY - oy; g.W = maxX - minX + 1; g.H = maxY - minY + 1;
            int stride = (g.W + 7) / 8;
            g.Bits = new byte[stride * g.H];
            for (int y = 0; y < g.H; y++)
                for (int x = 0; x < g.W; x++)
                    if (dark[minX + x, minY + y]) g.Bits[y * stride + x / 8] |= (byte)(0x80 >> (x % 8));
        }
        return g;
    }
}
