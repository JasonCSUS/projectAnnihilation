# projectAnnihilation
Engine functions
edit main.cpp to use whatever main game logic file you want to use, default is GameMain.cpp

what you need in the file:
it needs to take arguments: SDL_Window *window, SDL_Renderer *renderer
and call GameLoop with the window, renderer, an EntityManager, AnimationManager, and a callback function for updates
i.e. void UpdateGame(float deltaTime) {
    // Game logic here, e.g., updating AI, animations, physics, etc.
}

# engine functions

# AnimationManager.h

Contains the animation structure, this is my prebuilt tool for managing sprite sheets, animation frames, and sprite size. Ideally sprite size would be automatic but honestly leaving it manual allows for easier customization for both me and users

the functions you can call with an instanced animation manager:

# void AddAnimation(int entityId, Animation* animation); 
create an animation instance and pass it here along with an entityId. 
Animation's main 5 arguments are the sprite, the frame count, the animation time per frame, the sprite width, and sprite height
# Animation* GetAnimation(int entityId);
returns an animation pointer if you want to change any animation fields manually for some reason, or write new functions

# void SwapSprite(int entityId, Sprite* newSprite);
Sprites are actually just 4 sets of  2 int indexes for which part of a sprite sheet you want to use. kinda handy but you'll probably need to edit character.cpp yourself to fully use them
this function just swaps animation set you want to use. like walk/attack/idle etc

# void RenderEntity(SDL_Renderer* renderer, SDL_Texture* spriteSheet, Animation* animation, int x, int y, bool flip);
called automatically by gameloop through entitymanager. i'd advise reviewing what entitymanager is doing before manually calling this, since entitymanager is already calculating render position based on camera and world positions
