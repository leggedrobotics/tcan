#pragma once

#include <cstdint>

#include <boost/functional/hash.hpp>

namespace tcan_can {
    struct CanFrameIdentifier {
        explicit CanFrameIdentifier(uint32_t id, uint32_t msk = 0xffffffffu) : identifier(id), mask(msk) {};
        uint32_t identifier;
        uint32_t mask;

        bool operator==(const CanFrameIdentifier& rhs) const {
            return identifier == rhs.identifier && mask == rhs.mask;
        }
    };

    struct CanFrameIdentifierHasher {
        std::size_t operator()(const CanFrameIdentifier& matcher) const {
            std::size_t h = 0;
            boost::hash_combine(h, matcher.identifier);
            boost::hash_combine(h, matcher.mask);
            return h;
        };
    };
} // namespace tcan_can