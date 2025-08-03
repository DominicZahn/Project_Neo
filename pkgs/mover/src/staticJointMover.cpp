#include <cstdio>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rbdl_check_api_version(RBDL_API_VERSION);
  rbdl_print_version();
  printf("hello world mover package\n");
  return 0;
}
