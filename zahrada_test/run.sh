#!/usr/bin/env bash
# Logicke testy firmwaru zahradniho svetla na PC (bez hardwaru).
# Vytahne logiku z examples/zahrada_light/main.cpp (+ spolecny examples/zahrada_common/ZahradaNode.h), prelozi ji s napodobou MeshCore a spusti kontroly.
set -euo pipefail
cd "$(dirname "$0")"
python3 - <<'PY'
src = open('../examples/zahrada_light/main.cpp').read()
common = open('../examples/zahrada_common/ZahradaNode.h').read()
body = src[src.index('#ifndef LIGHT_ON_SECS'):src.index('StdRNG fast_rng;')]
open('extracted.h', 'w').write((common + body).replace('protected:', 'public:'))
PY
g++ -std=gnu++17 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -o test test.cpp
./test
