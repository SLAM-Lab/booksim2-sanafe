#ifndef _BOOKSIM_LIB_H_
#define _BOOKSIM_LIB_H_

#include "booksim_config.hpp"
BookSimConfig booksim_init(int argc, char **argv);
bool booksim_run(BookSimConfig const & config);
void booksim_close();

#endif // _BOOKSIM_LIB_H_