#include "solution.h"
using namespace std;

int main() {
    auto solution = std::make_shared<Solution>();
    solution->InitMap();
    while (solution->GetFrameInfo())
    {
        solution->AssaignTasks();
        //solution->PublishOrders();
    }

    return EXIT_SUCCESS;
}
