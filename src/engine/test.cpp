#include <spdlog/spdlog.h>

#include "physics_world.h"

int main()
{

    PhysicsWorld w;
    w.load_from_file("/home/ian/local_code/jelly_blend/test/world_data");
    std::cout << w.summary() << "\n";

    for (int i = 0; i < 100; ++i)
    {
        spdlog::info("cycle {}", i);
        w.simulate(1, 100, true);
    }

    return 0;
}