#pragma once
#include "cvBody.h"
#include <collision/cvContact.h>
#include <vector>

class cvCollisionAgent
{
public:
    void computeContacts();

public:
    cvBodyId m_bodyIds[2];

    // contact info
    std::vector<cvContact> m_contacts;
};
