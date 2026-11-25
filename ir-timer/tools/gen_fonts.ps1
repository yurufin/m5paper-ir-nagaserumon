# ir-timer の画面用フォント src/ui_fonts.h を作る(Windows の TTF → 1 ビットのビットマップ)
#   使い方: powershell -ExecutionPolicy Bypass -File tools\gen_fonts.ps1 [-Family "Noto Sans JP"]
#   入れる文字は gen_fonts.cs の Specs にある分だけ。画面の文言を増やしたらそこに足して作り直す
#   既定の Noto Sans JP は SIL Open Font License。Windows に無ければ Google Fonts から入れる
param([string]$Family = "Noto Sans JP")
$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
Add-Type -TypeDefinition (Get-Content -Raw -Encoding UTF8 (Join-Path $PSScriptRoot "gen_fonts.cs")) -ReferencedAssemblies System.Drawing
[UiFontGen]::Run($Family, (Join-Path $root "src\ui_fonts.h"))
"-> src\ui_fonts.h"
