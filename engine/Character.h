#ifndef CHARACTER_H
#define CHARACTER_H

// Generic sprite frame reference inside a sprite sheet.
// x and y are frame indices, not pixel coordinates.
struct Sprite {
    int x = 0;
    int y = 0;
};

#endif // CHARACTER_H