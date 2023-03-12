#include "solution.h"
using namespace std;

int main() {
    auto solution = std::make_shared<Solution>();
    solution->InitMap();
    while (solution->GetFrameInfo())
    {
        solution->AssaignTasks();
    }

    return EXIT_SUCCESS;
}
