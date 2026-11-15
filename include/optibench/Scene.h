#pragma once
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "Elements.h"
#include "Framebuffer.h"
#include "clip_fill.h"      // ClipRect

namespace ob {
    /**
     * The optical bench: a viewport plus every element on it.
     *
     * Typical use:
     *      Scene scene(ClipRect{40, 40, 760, 460});
     *      scene.emplace<LaserSource>("laser", Transform2D{...}, colors::Red);
     *      scene.emplace<LensElement>("lens", Transform2D{...}, 60.0);
     *      scene.render(fb);
     *
     * Animation: look elements up by name and change their `transform`, then call render() again.
     */
    class Scene {
    public:
        explicit Scene(const ClipRect& viewport) : viewport_(viewport) {}

        // appearance of the bench
        int gridSpacing = 40;
        Color marginColor = colors::BenchDark;      // outside the viewport (shows what clipping removed)
        Color panelColor{22, 26, 40};               // bench surface inside the viewport
        Color gridColor{32, 36, 50};

        // creates an element of type T in place and returns a reference to it
        template <class T, class... Args>
        T& emplace(Args&&... args) {
            auto owned = std::make_unique<T>(std::forward<Args>(args)...);
            T& ref = *owned;
            if constexpr (std::is_base_of_v<LaserSource, T>) sources_.push_back(&ref);
            elements_.push_back(std::move(owned));
            return ref;
        }

        // finds an element by name (nullptr if there is none)
        OpticalElement* find(const std::string& name);
        const OpticalElement* find(const std::string& name) const;

        // finds an element by name as a specific type (nullptr if missing or of another type)
        template <class T>
        T* findAs(const std::string& name) { return dynamic_cast<T*>(find(name)); }
        template <class T>
        const T* findAs(const std::string& name) const { return dynamic_cast<const T*>(find(name)); }

        const std::vector<std::unique_ptr<OpticalElement>>& elements() const { return elements_; }
        const std::vector<LaserSource*>& sources() const { return sources_; }
        const ClipRect& viewport() const { return viewport_; }

        // nearest thing a ray hits
        struct Intersection {
            Hit hit;
            const OpticalElement* element = nullptr;
        };
        /**
         * nearest element the ray hits.
         * Broad phase (geometry module): an element whose bounds() the ray misses, or enters only
         * beyond the best hit so far, is skipped without calling its intersect().
         */
        Intersection nearestHit(const Ray& ray) const;

        // the first solid GLASS element containing p (where a beam starting at p is travelling), or nullptr
        const OpticalElement* solidAt(const Vec2& p) const;

        /**
         * every pair of elements whose bodies overlap (geom::polygonsOverlap on their outlines;
         * touching is allowed). The tracer assumes glass never overlaps, so a valid bench has none.
         */
        std::vector<std::pair<const OpticalElement*, const OpticalElement*>> overlappingPairs() const;

        // paints the whole scene: panel, grid, elements, beams
        void render(Framebuffer& fb) const;

    private:
        ClipRect viewport_;
        std::vector<std::unique_ptr<OpticalElement>> elements_;
        std::vector<LaserSource*> sources_;     // non-owning views into elements_
    };
}
