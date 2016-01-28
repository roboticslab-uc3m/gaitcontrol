#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/spgait')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    spgait = RaveCreateModule(env,'spgait')
    print spgait.SendCommand('help')
finally:
    RaveDestroy()
