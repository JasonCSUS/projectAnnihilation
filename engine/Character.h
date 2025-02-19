#ifndef CHARACTER_H
#define CHARACTER_H
#include <SDL3/SDL.h>

struct Sprite {
    int x, y, w, h;
    int spriteW, spriteH; // Stores overall sprite dimensions

    // Constructor only takes width & height
    Sprite(int spriteW, int spriteH)
        : spriteW(spriteW), spriteH(spriteH), x(0), y(0), w(spriteW), h(spriteH) {}

    // Function to set sprite position dynamically
    void SetSprite(int row, int col) {
        x = col * spriteW;
        y = row * spriteH;
        w = spriteW;
        h = spriteH;
    }
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

void InitializeSprites();

void RenderCharacter(SDL_Renderer *renderer, SDL_Texture *spriteSheet, const Sprite &sprite, int x, int y, bool flip);

#endif