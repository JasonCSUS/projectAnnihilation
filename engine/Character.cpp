#include "Character.h"
#include <vector>

// Idle animations
std::vector<Sprite> idleUp    = { {0, 4}, {0, 4}, {0, 4}, {0, 4} };
std::vector<Sprite> idleRight = { {2, 4}, {2, 4}, {2, 4}, {2, 4} };
std::vector<Sprite> idleDown  = { {4, 4}, {4, 4}, {4, 4}, {4, 4} };

// Special animations
std::vector<Sprite> specialUp    = { {1, 0}, {1, 0}, {1, 0}, {1, 0} };
std::vector<Sprite> specialRight = { {3, 0}, {3, 0}, {3, 0}, {3, 0} };
std::vector<Sprite> specialDown  = { {5, 0}, {5, 0}, {5, 0}, {5, 0} };

// Attack animations
std::vector<Sprite> attackUp    = { {1, 4}, {1, 3}, {1, 2}, {1, 1} };
std::vector<Sprite> attackRight = { {3, 4}, {3, 3}, {3, 2}, {3, 1} };
std::vector<Sprite> attackDown  = { {5, 4}, {5, 3}, {5, 2}, {5, 1} };

// Walk animations
std::vector<Sprite> walkUp    = { {0, 3}, {0, 2}, {0, 1}, {0, 0} };
std::vector<Sprite> walkRight = { {2, 3}, {2, 2}, {2, 1}, {2, 0} };
std::vector<Sprite> walkDown  = { {4, 3}, {4, 2}, {4, 1}, {4, 0} };
