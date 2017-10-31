#pragma once

#include <vector>
#include <unordered_set>

#include <core/collection/cvFreeList.h>
#include <simulation/cvBody.h>


namespace std
{
    template<> struct hash<cvBodyId>
    {
        std::size_t operator()(const cvBodyId& h) const
        {
            return hash<uint16_t>{}(h.getVal());
        }
    };
}

class cvBodyManager
{
public:
    typedef cvFreeList<cvBody, cvBodyId> BodyBuffer;
    typedef std::unordered_set<cvBodyId>::const_iterator bodyIter;
    class AllocatedBodyIter : public bodyIter
    {
    public:
        AllocatedBodyIter(const std::unordered_set<cvBodyId>& bodyBuffer)
            : m_allocatedBodies(bodyBuffer), bodyIter(bodyBuffer.begin()) { }
        bool isValid() const {return m_allocatedBodies.end() != (bodyIter&)(*this);}
        const std::unordered_set<cvBodyId>& m_allocatedBodies;
    };
public:
    cvBodyManager();
    cvBodyId allocate(const cvBodyCInfo& cinfo);
    void free(cvBodyId bodyId);
    const cvBody& getBody(cvBodyId bodyId) const;
    cvBody& accessBody(cvBodyId bodyId);


    const BodyBuffer& getBodyBuffer() const {return m_bodyBuffer;}
    AllocatedBodyIter getBodyIter() const {return AllocatedBodyIter(m_allocated);}

private:
    cvFreeList<cvBody, cvBodyId> m_bodyBuffer;
    std::unordered_set<cvBodyId> m_allocated;
};
