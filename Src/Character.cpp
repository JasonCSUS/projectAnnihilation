#include "Character.h"
#include <SDL3/SDL.h>
#include <iostream>

Sprite idleUp[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };
Sprite idleRight[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };
Sprite idleDown[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };

Sprite dashUp[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };
Sprite dashRight[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };
Sprite dashDown[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };

Sprite attackUp[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };
Sprite attackRight[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };
Sprite attackDown[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };

Sprite walkUp[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };
Sprite walkRight[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };
Sprite walkDown[4] = { {64, 64}, {64, 64}, {64, 64}, {64, 64} };


// Set positions dynamically using `SetSprite(row, col)`
void InitializeSprites() {
    for (int i = 0; i < 4; i++) {
        idleUp[i].SetSprite(0, 0);    
        idleRight[i].SetSprite(0, 2);
        idleDown[i].SetSprite(0, 4);

        dashUp[i].SetSprite(4, 1);   
        dashRight[i].SetSprite(4, 3);
        dashDown[i].SetSprite(4, 5);

        attackUp[i].SetSprite(i, 1);
        attackRight[i].SetSprite(i, 3);
        attackDown[i].SetSprite(i, 5);

        walkUp[i].SetSprite(i+1, 0);
        walkRight[i].SetSprite(i+1, 2);
        walkDown[i].SetSprite(i+1, 4);
    }
}

void RenderCharacter(SDL_Renderer *renderer, SDL_Texture *spriteSheet, const Sprite &sprite, int x, int y, bool flip) {
    SDL_FRect src = {static_cast<float>(sprite.x), static_cast<float>(sprite.y), static_cast<float>(sprite.w), static_cast<float>(sprite.h)};
    SDL_FRect dst = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(sprite.w), static_cast<float>(sprite.h)};
    SDL_RenderTextureRotated(renderer, spriteSheet, &src, &dst, 0.0, NULL, flip ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE);
}

