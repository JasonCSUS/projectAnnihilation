#include "Character.h"
#include <SDL3/SDL.h>
#include <iostream>

Sprite idleUp[4] = { {0, 0}, {0, 0}, {0, 0}, {0, 0} };
Sprite idleRight[4] = { {2, 0}, {2, 0}, {2, 0}, {2, 0} };
Sprite idleDown[4] = { {4, 0}, {4, 0}, {4, 0}, {4, 0} };

Sprite specialUp[4] = { {1, 4}, {1, 4}, {1, 4}, {1, 4} };
Sprite specialRight[4] = { {3, 4}, {3, 4}, {3, 4}, {3, 4} };
Sprite SpecialDown[4] = { {5, 4}, {5, 4}, {5, 4}, {5, 4} };

Sprite attackUp[4] = { {1, 0}, {1, 1}, {1, 2}, {1, 3} };
Sprite attackRight[4] = { {3, 0}, {3, 1}, {3, 2}, {3, 3} };
Sprite attackDown[4] = { {5, 0}, {5, 1}, {5, 2}, {5, 3} };

Sprite walkUp[4] = { {0, 1}, {0, 2}, {0, 3}, {0, 4} };
Sprite walkRight[4] = { {2, 1}, {2, 2}, {2, 3}, {2, 4} };
Sprite walkDown[4] = { {4, 1}, {4, 2}, {4, 3}, {4, 4} };

