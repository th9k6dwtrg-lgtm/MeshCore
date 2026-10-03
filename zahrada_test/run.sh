#!/usr/bin/env bash
# Logicke testy firmwaru zahradniho svetla na PC (bez hardwaru).
# Vytahne logiku z examples/zahrada_light/main.cpp, prelozi ji s napodobou MeshCore a spusti kontroly.
set -euo pipefail
cd "$(dirname "$0")"
python3 - <<'PY'
src = open('../examples/zahrada_light/main.cpp').read()
body = src[src.index('#ifndef LIGHT_ON_SECS'):src.index('StdRNG fast_rng;')].replace('protected:', 'public:')
open('extracted.h', 'w').write(body)
PY
g++ -std=gnu++17 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -o test test.cpp
./test
