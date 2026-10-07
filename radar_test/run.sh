#!/usr/bin/env bash
# Logicke testy firmwaru radaru na PC (bez hardwaru).
# Vytahne logiku z examples/radar_alarm/main.cpp, prelozi ji s napodobou MeshCore (zahrada_test/mocks.h) a spusti kontroly.
set -euo pipefail
cd "$(dirname "$0")"
python3 - <<'PY'
src = open('../examples/radar_alarm/main.cpp').read()
body = src[src.index('#ifndef LIGHT_PULSE_SECS'):src.index('StdRNG fast_rng;')].replace('protected:', 'public:')
open('extracted.h', 'w').write(body)
PY
g++ -std=gnu++17 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -I ../zahrada_test -o test test.cpp
./test
