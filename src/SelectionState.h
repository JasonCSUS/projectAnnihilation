#ifndef SELECTIONSTATE_H
#define SELECTIONSTATE_H

#include <SDL3/SDL.h>
#include <vector>

class EntityManager;

class SelectionState {
public:
    void Clear();

    void SetSingle(int entityId);
    void SetSelection(const std::vector<int>& entityIds);
    void AddSelection(const std::vector<int>& entityIds);

    bool Empty() const;
    bool IsSelected(int entityId) const;

    const std::vector<int>& GetSelectedIds() const;
    int GetPrimarySelectedId() const;
    void SetPrimarySelectedId(int entityId);

    void RemoveMissing(const EntityManager& entityManager);

    void BeginMarquee(float screenX, float screenY);
    void UpdateMarquee(float screenX, float screenY);
    void EndMarquee();

    bool IsMarqueeActive() const;
    bool IsMarqueeVisible() const;
    const SDL_FRect& GetMarqueeRect() const;

private:
    void NormalizeSelection();
    void UpdatePrimaryFromSelection();

private:
    std::vector<int> selectedIds;
    int primarySelectedId = -1;

    bool marqueeActive = false;
    bool marqueeVisible = false;
    float marqueeStartX = 0.0f;
    float marqueeStartY = 0.0f;
    SDL_FRect marqueeRect{0.0f, 0.0f, 0.0f, 0.0f};
};

#endif
