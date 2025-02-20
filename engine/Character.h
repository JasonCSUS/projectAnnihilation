#ifndef CHARACTER_H
#define CHARACTER_H
#include <SDL3/SDL.h>

struct Sprite {
    int x, y;
};

extern Sprite idleUp[4];
extern Sprite idleRight[4];
extern Sprite idleDown[4];
extern Sprite dashUp[4];
extern Sprite dashRight[4];
extern Sprite dashDown[4];
extern Sprite attackUp[4];
extern Sprite attackRight[4];
extern Sprite attackDown[4];
extern Sprite walkUp[4];
extern Sprite walkRight[4];
extern Sprite walkDown[4];

#endif