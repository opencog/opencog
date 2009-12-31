This module provides a RESTful interface to OpenCog

Gama implemented such an interface a while ago, but it relied on the Http
sockets of the CSockets library that's since been removed as a dependency from
OpenCog. This implementation uses the Mongoose embeddable http server:

http://code.google.com/p/mongoose/

(Although the code has been included in the OpenCog code base due to it being so
light weight. It is distributed under the MIT license).


