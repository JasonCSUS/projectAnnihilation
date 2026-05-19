#include "SelectionState.h"
#include "../engine/EntityManager.h"

#include <algorithm>
#include <unordered_set>

void SelectionState::Clear() {
    selectedIds.clear();
    primarySelectedId = -1;
    marqueeActive = false;
    marqueeVisible = false;
    marqueeRect = {0.0f, 0.0f, 0.0f, 0.0f};
}

void SelectionState::SetSingle(int entityId) {
    selectedIds.clear();
    if (entityId >= 0) {
        selectedIds.push_back(entityId);
        primarySelectedId = entityId;
    } else {
        primarySelectedId = -1;
    }
}

void SelectionState::SetSelection(const std::vector<int>& entityIds) {
    selectedIds = entityIds;
    NormalizeSelection();
    UpdatePrimaryFromSelection();
}

void SelectionState::AddSelection(const std::vector<int>& entityIds) {
    selectedIds.insert(selectedIds.end(), entityIds.begin(), entityIds.end());
    NormalizeSelection();
    UpdatePrimaryFromSelection();
}

bool SelectionState::Empty() const {
    return selectedIds.empty();
}

bool SelectionState::IsSelected(int entityId) const {
    return std::find(selectedIds.begin(), selectedIds.end(), entityId) != selectedIds.end();
}

const std::vector<int>& SelectionState::GetSelectedIds() const {
    return selectedIds;
}

int SelectionState::GetPrimarySelectedId() const {
    return primarySelectedId;
}

void SelectionState::SetPrimarySelectedId(int entityId) {
    if (IsSelected(entityId)) {
        primarySelectedId = entityId;
    }
}

void SelectionState::RemoveMissing(const EntityManager& entityManager) {
    std::unordered_set<int> liveIds;
    liveIds.reserve(entityManager.entities.size());

    for (const auto& entity : entityManager.entities) {
        if (!entity.isDead) {
            liveIds.insert(entity.id);
        }
    }

    selectedIds.erase(
        std::remove_if(
            selectedIds.begin(),
            selectedIds.end(),
            [&](int id) { return liveIds.find(id) == liveIds.end(); }
        ),
        selectedIds.end()
    );

    UpdatePrimaryFromSelection();
}

void SelectionState::BeginMarquee(float screenX, float screenY) {
    marqueeActive = true;
    marqueeVisible = false;
    marqueeStartX = screenX;
    marqueeStartY = screenY;
    marqueeRect = {screenX, screenY, 0.0f, 0.0f};
}

void SelectionState::UpdateMarquee(float screenX, float screenY) {
    if (!marqueeActive) {
        return;
    }

    const float minX = std::min(marqueeStartX, screenX);
    const float minY = std::min(marqueeStartY, screenY);
    const float maxX = std::max(marqueeStartX, screenX);
    const float maxY = std::max(marqueeStartY, screenY);

    marqueeRect.x = minX;
    marqueeRect.y = minY;
    marqueeRect.w = maxX - minX;
    marqueeRect.h = maxY - minY;

    marqueeVisible = (marqueeRect.w >= 4.0f || marqueeRect.h >= 4.0f);
}

void SelectionState::EndMarquee() {
    marqueeActive = false;
    marqueeVisible = false;
    marqueeRect = {0.0f, 0.0f, 0.0f, 0.0f};
}

bool SelectionState::IsMarqueeActive() const {
    return marqueeActive;
}

bool SelectionState::IsMarqueeVisible() const {
    return marqueeVisible;
}

const SDL_FRect& SelectionState::GetMarqueeRect() const {
    return marqueeRect;
}

void SelectionState::NormalizeSelection() {
    std::vector<int> normalized;
    normalized.reserve(selectedIds.size());

    std::unordered_set<int> seen;
    seen.reserve(selectedIds.size());

    for (int id : selectedIds) {
        if (id < 0) {
            continue;
        }
        if (seen.insert(id).second) {
            normalized.push_back(id);
        }
    }

    selectedIds = std::move(normalized);
}

void SelectionState::UpdatePrimaryFromSelection() {
    if (selectedIds.empty()) {
        primarySelectedId = -1;
        return;
    }

    if (!IsSelected(primarySelectedId)) {
        primarySelectedId = selectedIds.front();
    }
}
