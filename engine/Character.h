#ifndef CHARACTER_H
#define CHARACTER_H
#include <SDL3/SDL.h>
#include <vector>

struct Sprite {
    int x, y;
};

extern std::vector<Sprite> idleUp;
extern std::vector<Sprite> idleRight;
extern std::vector<Sprite> idleDown;
extern std::vector<Sprite> dashUp;
extern std::vector<Sprite> dashRight;
extern std::vector<Sprite> dashDown;
extern std::vector<Sprite> attackUp;
extern std::vector<Sprite> attackRight;
extern std::vector<Sprite> attackDown;
extern std::vector<Sprite> walkUp;
extern std::vector<Sprite> walkRight;
extern std::vector<Sprite> walkDown;

#endif