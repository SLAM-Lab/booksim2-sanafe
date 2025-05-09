#ifndef _BOOKSIM_LIB_H_
#define _BOOKSIM_LIB_H_

#include "booksim_config.hpp"
void booksim_init(int argc, char **argv);
bool booksim_run(BookSimConfig const & config);

#endif // _BOOKSIM_LIB_H_