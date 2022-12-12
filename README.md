## JaiBullet

The default C wrapper for bullet is also some extreme weird shit that wraps a "physics server" around a C library and is intertwined with graphical functionality.

Seems completely ridiculous to write code for a game this way, so I tried generating CPP bindings for a simplified version of the library which was much too complicated for the current bindings generator.

And finally I found a C wrapper online for zbullet that wraps most of the useful functionality (missing a lot) and this is what is currently binded to.

I also generated helper methods for each method that accepts a pointer to a vector/matrix/float rather than the actual object.
