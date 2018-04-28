#pragma once

#include <vector>
#include <unordered_set>

#include <core/collection/cvFreeList.h>
#include <world/cvBody.h>

class cvBodyManager
{
public:
    typedef cvFreeList<cvBody, cvBodyId> BodyBuffer;
    typedef std::unordered_set<cvBodyId>::const_iterator bodyIter;
    class AllocatedBodyIter : public bodyIter
    {
    public:
        AllocatedBodyIter(const std::unordered_set<cvBodyId>& bodyBuffer)
            : bodyIter(bodyBuffer.begin()), m_allocatedBodies(bodyBuffer)  { }
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
