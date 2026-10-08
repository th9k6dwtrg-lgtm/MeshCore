#!/usr/bin/env bash
# Logicke testy firmwaru radaru na PC (bez hardwaru).
# Vytahne logiku z examples/radar_alarm/main.cpp (+ spolecny examples/zahrada_common/ZahradaNode.h), prelozi ji s napodobou MeshCore (zahrada_test/mocks.h) a spusti kontroly.
set -euo pipefail
cd "$(dirname "$0")"
python3 - <<'PY'
src = open('../examples/radar_alarm/main.cpp').read()
common = open('../examples/zahrada_common/ZahradaNode.h').read()
body = src[src.index('#ifndef LIGHT_PULSE_SECS'):src.index('StdRNG fast_rng;')]
open('extracted.h', 'w').write((common + body).replace('protected:', 'public:'))
PY
g++ -std=gnu++17 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -I ../zahrada_test -DRADAR_STARTUP_SECS=30 -o test test.cpp
./test
# znovu s vychozim ustalovanim 5 minut
g++ -std=gnu++17 -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -I ../zahrada_test -DRADAR_STARTUP_SECS=300 -o test test.cpp
./test
