#pragma once

#include <exception>

class MeshGenExceedIntMax : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "execeed int max";
    }
};

class MeshGenOutOfMem : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "tetgen run out of memory";
    }
};

class MeshGenKnowBug : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "tetgen encounters a known bug";
    }
};

class MeshGenSelfIntersection : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "tetgen detect self intersections";
    }
};

class MeshGenSmallFeature : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "tetgen detect a very small input feature";
    }
};

class MeshGenUnknown : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "unknown tetgen error";
    }
};

class BlMeshModified : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "blender object mesh is modified";
    }
};

class BlObjectMissing : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "no blender object matching name";
    }
};

class SIMBlowUp : public std::exception
{
  public:
    const char *what() const noexcept override
    {
        return "simulation does not converge, softbody blowing up...";
    }
};