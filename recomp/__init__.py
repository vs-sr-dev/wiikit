"""wiikit layer 4: static recompilation of Gekko code to C++.

    program   units, entry points, labels and switch tables of an image
    emit      one instruction -> C++ against wiikit/runtime/ppc.h
    __main__  python -m wiikit.recomp GAME.elf --out DIR
"""
