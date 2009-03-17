#include "HPASearch.h"
#include <cmath>
#include <map>
#include "util/Logger.h"
#include "QuadTree.h"

namespace Spatial {

  HPASearch::Level::Level( LocalSpaceMap2D* map, unsigned int level, unsigned int maximumClusters ) {
    this->map = map;
    this->level = level;
    this->vertexCounter = 0;
    this->numberOfCols = maximumClusters * level;
    this->numberOfRows = maximumClusters * level;
    this->clusterDimension.width = ( this->map->xMax( ) - this->map->xMin( ) ) / numberOfCols;
    this->clusterDimension.height = ( this->map->yMax( ) - this->map->yMin( ) ) / numberOfRows;
    this->abstractGraph = new Graph( this->map->xDim( ) * this->map->yDim( ) );
    this->needsUpdate = true;
  }

  HPASearch::Level::~Level( void ) {
    delete abstractGraph;
  }

  const HPASearch::Graph& HPASearch::Level::getAbstractGraph( void ) {
    return *this->abstractGraph;
  }

  float HPASearch::Level::getClusterWidth( void ) {
    return this->clusterDimension.width;
  }
  
  float HPASearch::Level::getClusterHeight( void ) {
    return this->clusterDimension.height;
  }

  unsigned int HPASearch::Level::getLevel( void ) {
    return this->level;
  }

  unsigned int HPASearch::Level::getClusterId( const GridPoint& gridPoint ) const {
    int row = gridPoint.second / ( map->yDim( ) / numberOfRows );
    int col = gridPoint.first / ( map->xDim( ) / numberOfCols );
    return ( ( row * numberOfRows ) + col );
  }
  
  void HPASearch::Level::buildClusters( void ) {

    unsigned int clusterCols = 
      static_cast<unsigned int>( clusterDimension.width / this->map->xGridWidth( ) );
    unsigned int clusterRows = 
      static_cast<unsigned int>( clusterDimension.height / this->map->yGridWidth( ) );

    unsigned int row;
    unsigned int col;

    // build all cluster entrances
    for( row = 0; row < numberOfRows; ++row ) {		
      for( col = 0; col < numberOfCols; ++col ) {
	buildEntrance( row, col, true );
	buildEntrance( row, col, false );
      } // for
    } // for

    boost::property_map<Graph, HPASearch::VertexPosition>::type position = 
      boost::get( HPASearch::VertexPosition( ), *this->abstractGraph );
	

    for( row = 0; row < numberOfRows; ++row ) {		
      for( col = 0; col < numberOfCols; ++col ) {	
	unsigned int rowClusterOffset = clusterRows * row;
	unsigned int colClusterOffset = clusterCols * col;

	unsigned int beforeVertexCounter = this->vertexCounter;
	QuadTree( this, GridPoint( colClusterOffset, rowClusterOffset ), clusterCols, this->abstractGraph ).connectEdges( );		
	unsigned int numberOfClusterVertices = this->vertexCounter - beforeVertexCounter;

	unsigned int clusterId = getClusterId( GridPoint( colClusterOffset, rowClusterOffset ) );
	clustersVertexRange[ clusterId ].first = beforeVertexCounter;
	clustersVertexRange[ clusterId ].second = numberOfClusterVertices;

	std::vector< GridPoint >& nearestEntrances = 
	  clusterEntrances[ clusterId ];

	unsigned int i;
	unsigned int j;
	if ( numberOfClusterVertices > 0 ) {
	  
	  std::vector<float> distances(nearestEntrances.size( ));
	  std::vector<unsigned int> nearestVertex(nearestEntrances.size( ));
	  std::fill_n( distances.begin( ), nearestEntrances.size( ),  ( map->xDim( ) * map->yDim( ) ) );
	  
	  for( j = 0; j < numberOfClusterVertices; ++j ) {
	    unsigned int vertexId = beforeVertexCounter + j;
	    Math::Vector2 position = boost::get( HPASearch::VertexPosition( ), *this->abstractGraph, vertexId );
	    
	    for( i = 0; i < nearestEntrances.size( ); ++i ) {
	      Math::Vector2 entrancePosition = boost::get( HPASearch::VertexPosition( ), 
						     *this->abstractGraph, 
						     this->graphVertices[ nearestEntrances[ i ] ] );
	      
	      float candidateDistance = ( position - entrancePosition ).length( );
	      
	      if ( candidateDistance < distances[ i ] ) {
		distances[ i ] = candidateDistance;
		nearestVertex[ i ] = vertexId;
	      } // if
	      
	    } // for
	  } // for
	  
	  // connect all entrances to the inner graph
	  for( i = 0; i < nearestEntrances.size( ); ++i ) {
	    boost::add_edge( this->graphVertices[ nearestEntrances[ i ] ],
			     nearestVertex[ i ], distances[ i ], *this->abstractGraph );	  
	  } // for

	} else {
	  // there is no obstacle inside cluster, so
	  // inter-connect all entrances 
	  for( i = 0; i < nearestEntrances.size( ); ++i ) {
	    for( j = i+1; j < nearestEntrances.size( ); ++j ) {

	      unsigned int vertex1 = this->graphVertices[ nearestEntrances[ i ] ];
	      unsigned int vertex2 = this->graphVertices[ nearestEntrances[ j ] ];
	      Math::Vector2 position1 = boost::get( HPASearch::VertexPosition( ), 
					      *this->abstractGraph, 
					      vertex1 );

	      Math::Vector2 position2 = boost::get( HPASearch::VertexPosition( ), 
					      *this->abstractGraph, 
					      vertex2 );

	      boost::add_edge( vertex1, vertex2, 
			       ( position1 - position2 ).length( ), *this->abstractGraph );	      
	    } // for
	  } // for

	} // else

      } // for
    } // for
    
    // connect all intra cluster entrances
    unsigned int i;
    for( i = 0; i < this->entrances.size( ); ++i ) {
      unsigned int vertex1 = this->graphVertices[ this->entrances[ i ].first ];
      unsigned int vertex2 = this->graphVertices[ this->entrances[ i ].second ];
      boost::add_edge( vertex1, vertex2, clusterDimension.width, *this->abstractGraph );
    } // for

    // make a reduced copy of the graph
    if ( boost::num_vertices( *this->abstractGraph ) / 2 > this->vertexCounter ) {

      Graph* oldGraph = this->abstractGraph;
      this->abstractGraph = new Graph( this->vertexCounter );

      boost::property_map<Graph, HPASearch::VertexPosition>::type position = 
	boost::get( HPASearch::VertexPosition( ), *this->abstractGraph );

      // copy vertices
      unsigned int i;
      for( i = 0; i < this->vertexCounter; ++i ) {
	Math::Vector2 realPosition = 
	  boost::get( HPASearch::VertexPosition( ), *oldGraph, i );
	boost::put( position, i, realPosition );
      } // if
      
      // copy edges
      EdgeIterator it = boost::edges( *oldGraph );
      while( it.first != it.second ) {

	VertexDescriptor v1 = boost::source( *it.first, *oldGraph );
	VertexDescriptor v2 = boost::target( *it.first, *oldGraph );
	
	boost::add_edge( v1, v2, 0, *this->abstractGraph );
	++it.first;
      } // while
      
      delete oldGraph;
      
    } // if

    
    
  }

  void HPASearch::Level::setupVertex( const GridPoint& gridPoint ) {
    boost::property_map<Graph, HPASearch::VertexPosition>::type position = 
      boost::get( HPASearch::VertexPosition( ), *this->abstractGraph );

    Point realPosition = map->unsnap( gridPoint );
    Math::Vector2 cellCenterPosition( realPosition.first, realPosition.second );
    this->graphVertices[ gridPoint ] = vertexCounter;
    boost::put( position, vertexCounter, cellCenterPosition );
    vertexCounter++;
  }
  
  void HPASearch::Level::buildEntrance( unsigned int row, unsigned int col, bool horizontal ) {
    float numberOfCols = clusterDimension.width / map->xGridWidth( );
    float numberOfRows = clusterDimension.height / map->yGridWidth( );

    if ( horizontal ) {
      unsigned int topLeftCluster1Col = static_cast<unsigned int>
	( ( col * numberOfCols ) + numberOfCols - 1 );
      unsigned int topLeftCluster1Row = static_cast<unsigned int>( row * numberOfRows );
      unsigned int topRightCluster2Col = topLeftCluster1Col + 1;
      unsigned int topRightCluster2Row = topLeftCluster1Row;

		
      std::vector< std::pair<GridPoint, GridPoint> > localEntrances;
      unsigned int i;

      for( i = 0; i < numberOfRows; ++i ) {
	GridPoint cell1( topLeftCluster1Col, topLeftCluster1Row+i );
	GridPoint cell2( topRightCluster2Col, topRightCluster2Row+i );
	
	if ( !map->gridIllegal( cell1 ) && !map->gridIllegal( cell2 ) ) {
	  localEntrances.push_back( std::pair<GridPoint, GridPoint>( cell1, cell2 ) );
	} else if ( localEntrances.size( ) > 0 ) {
	  unsigned int selectedIndex = ( localEntrances.size( ) ) / 2;
	  std::pair<GridPoint, GridPoint>& entrance = localEntrances[ selectedIndex ];
	  this->entrances.push_back( entrance );
	  
	  this->clusterEntrances[ getClusterId( entrance.first ) ].push_back( entrance.first );
	  this->clusterEntrances[ getClusterId( entrance.second ) ].push_back( entrance.second );

	  setupVertex( entrance.first );
	  setupVertex( entrance.second );
	} // else
      } // for
      
      if ( localEntrances.size( ) > 0 ) {
	unsigned int selectedIndex = ( localEntrances.size( ) ) / 2;
	std::pair<GridPoint, GridPoint>& entrance = localEntrances[ selectedIndex ];
	this->entrances.push_back( entrance );

	this->clusterEntrances[ getClusterId( entrance.first ) ].push_back( entrance.first );
	this->clusterEntrances[ getClusterId( entrance.second ) ].push_back( entrance.second );

	setupVertex( entrance.first );
	setupVertex( entrance.second );
      } // if
    } else {
      unsigned int bottomLeftCluster1Col = static_cast<unsigned int>( col * numberOfCols );
      unsigned int bottomLeftCluster1Row = static_cast<unsigned int>
	( ( row * numberOfRows ) + numberOfRows - 1 );      
      unsigned int topLeftCluster2Col = bottomLeftCluster1Col;
      unsigned int topLeftCluster2Row = bottomLeftCluster1Row+1;
		
      std::vector< std::pair<GridPoint, GridPoint > > localEntrances;
      unsigned int i;
      for( i = 0; i < numberOfRows; ++i ) {
	GridPoint cell1( bottomLeftCluster1Col+i, bottomLeftCluster1Row );
	GridPoint cell2( topLeftCluster2Col+i, topLeftCluster2Row );
	
	if ( !map->gridIllegal( cell1 ) && !map->gridIllegal( cell2 ) ) {
	  localEntrances.push_back( std::pair<GridPoint, GridPoint>( cell1, cell2 ) );
	} else if ( localEntrances.size( ) > 0 ) {
	  unsigned int selectedIndex = ( localEntrances.size( ) ) / 2;
	  std::pair<GridPoint, GridPoint>& entrance = localEntrances[ selectedIndex ];
	  this->entrances.push_back( entrance );

	  this->clusterEntrances[ getClusterId( entrance.first ) ].push_back( entrance.first );
	  this->clusterEntrances[ getClusterId( entrance.second ) ].push_back( entrance.second );

	  setupVertex( entrance.first );
	  setupVertex( entrance.second );
	} // else

      } // for

      if ( localEntrances.size( ) > 0 ) {
	unsigned int selectedIndex = ( localEntrances.size( ) ) / 2;
	std::pair<GridPoint, GridPoint>& entrance = localEntrances[ selectedIndex ];
	this->entrances.push_back( entrance );

	this->clusterEntrances[ getClusterId( entrance.first ) ].push_back( entrance.first );
	this->clusterEntrances[ getClusterId( entrance.second ) ].push_back( entrance.second );
	  
	setupVertex( entrance.first );
	setupVertex( entrance.second );
      } // if

    } // else      
    
  }
  
  GridPoint HPASearch::Level::getNearestEntrance( unsigned int clusterId, const Math::Vector2& position ) {
    std::vector<GridPoint>& clusterEntrances = this->clusterEntrances[ clusterId ];

    GridPoint response;
    unsigned int i;
    float distance = ( map->xMax( ) - map->xMin( ) ) * ( map->yMax( ) - map->yMin( ) );
    for ( i = 0; i < clusterEntrances.size( ); ++i ) {
      Math::Vector2 entrancePosition = boost::get( HPASearch::VertexPosition( ), *this->abstractGraph, this->graphVertices[ clusterEntrances[ i ] ] );
      
      float candidateDistance = ( entrancePosition - position ).length( );
      if ( candidateDistance < distance ) {
	distance = candidateDistance;
	response = clusterEntrances[ i ];
      } // if
      
    } // for

    return response;
  }


  GridPoint HPASearch::Level::getNearestVertex( unsigned int clusterId, const Math::Vector2& position ) {
    unsigned int startRange = clustersVertexRange[ clusterId ].first;
    unsigned int numberOfVertices = clustersVertexRange[ clusterId ].second;
    
    GridPoint response;    

    float distance = clusterDimension.width * clusterDimension.height;
    unsigned int i;
    for( i = 0; i < numberOfVertices; ++i ) {
      unsigned int vertexId = startRange + i;

      Math::Vector2 vertexPosition = 
	boost::get( HPASearch::VertexPosition( ), *this->abstractGraph, vertexId );
      float candidateDistance = ( position - vertexPosition ).length( );
      if ( candidateDistance < distance ) {
	distance = candidateDistance;

	response = map->snap( Point( vertexPosition.x, vertexPosition.y ) );
      } // if
    } // for

    return response;
  }

  bool HPASearch::Level::processPath( const Math::Vector2& startPoint, const Math::Vector2& endPoint ) throw( opencog::RuntimeException ) {        

    this->processedPath.clear( );

    if ( this->needsUpdate ) {
      buildClusters( );
      this->needsUpdate = false;
    } // if
    
    bool invalidStartPoint = map->illegal( Point( startPoint.x, startPoint.y ) );
    bool invalidEndPoint = map->illegal( Point( endPoint.x, endPoint.y ) );

    if ( invalidStartPoint || invalidEndPoint ) {
      if ( invalidStartPoint ) {
	logger().log(opencog::Logger::DEBUG, ( "HPASearch - invalid start point [" + startPoint.toString( ) + "]" ).c_str( ) );
      } // if
      if ( invalidEndPoint ) {
	logger().log(opencog::Logger::DEBUG, ( "HPASearch - invalid end point [" + endPoint.toString( ) + "]" ).c_str( ) );	
      } // if
      return false;
    } // if    
    
    unsigned int cluster1Id = getClusterId( map->snap( Point( startPoint.x, startPoint.y ) ) );
    unsigned int cluster2Id = getClusterId( map->snap( Point( endPoint.x, endPoint.y ) ) );

    unsigned int cluster1NumberOfVertices = clustersVertexRange[ cluster1Id ].second;
    unsigned int cluster2NumberOfVertices = clustersVertexRange[ cluster2Id ].second;

    unsigned int startWaypoint;
    unsigned int endWaypoint;

    if ( cluster1Id == cluster2Id && cluster1NumberOfVertices == 0 ) {
      float distanceFromStartToGoal = ( startPoint - endPoint ).length( );

      // 0,1% of the map width
      float distanceTreshold = ( map->xMax( ) - map->xMin( ) ) * 0.001;

      if ( distanceFromStartToGoal > distanceTreshold ) {
	// ignore start point this->processedPath.push_back( startPoint );
	this->processedPath.push_back( endPoint );
      } else {
	logger().log(opencog::Logger::INFO, ( "HPASearch - start and end points are the same [" + startPoint.toString( ) + "]" ).c_str( ) );
      } // else

      return true;
    } else {
      if ( cluster1NumberOfVertices == 0  ) {
	startWaypoint = this->graphVertices[ getNearestEntrance( cluster1Id, endPoint ) ];
      } else {
	startWaypoint = this->graphVertices[ getNearestVertex( cluster1Id, startPoint ) ];
      } // else

      if ( cluster2NumberOfVertices == 0 ) {
	endWaypoint = this->graphVertices[ getNearestEntrance( cluster2Id, startPoint ) ];
      } else {
	endWaypoint = this->graphVertices[ getNearestVertex( cluster2Id, endPoint ) ];
      } // else


      // find a path inter-clusters
      std::vector< VertexDescriptor > predecessors( boost::num_vertices( *this->abstractGraph ) );
      std::vector< float > distances( boost::num_vertices( *this->abstractGraph ) );

      try {
	
	AStarDistanceHeuristic heuristic( boost::get( HPASearch::VertexPosition( ), *this->abstractGraph, endWaypoint ), *this->abstractGraph );
	
	boost::astar_search( *this->abstractGraph, 
			     startWaypoint, heuristic,
			     boost::predecessor_map( &predecessors[0] ).
			     distance_map( &distances[0] ).
			     visitor( AStarGoalVisitor( endWaypoint ) ) );

	// ignore start point this->processedPath.push_back( startPoint );
	this->processedPath.push_back( endPoint );

	return false;
      } catch( FoundGoal& ex ) {	
	unsigned int i;

	if ( endPoint != boost::get( HPASearch::VertexPosition( ), *this->abstractGraph, endWaypoint ) ) {
	  this->processedPath.push_back( endPoint );
	} // if

	bool running = true;
	for( i = endWaypoint; running ; i = predecessors[ i ] ) {
	  Math::Vector2 wayPoint = boost::get( HPASearch::VertexPosition( ), *this->abstractGraph, i );
	  this->processedPath.insert( this->processedPath.begin( ), wayPoint );	
	  running = ( predecessors[ i ] != i );
	} // for

	if ( startPoint != boost::get( HPASearch::VertexPosition( ), *this->abstractGraph, startWaypoint ) ) {
	  this->processedPath.insert( this->processedPath.begin( ), startPoint );	
	} // if
	
      } // catch
    } // else

    smoothPath( );
    smoothTriangulate( );

    return true;
  }

  const std::vector<Math::Vector2>& HPASearch::Level::getProcessedPath( void ) const {
    return this->processedPath;
  }


  void HPASearch::Level::smoothPath( void ) {
    if ( this->processedPath.size( ) < 2 ) {
      return;
    } // if

    float tolerance = 1.0;

    std::vector<Math::Vector2> smoothPath;
    
    logger().log(opencog::Logger::INFO, "HPASearch - smoothing path. (%d) points.",
        this->processedPath.size( ) );
    
    std::vector<Math::Vector2>::iterator it = this->processedPath.begin( );
    
    smoothPath.push_back( *it );

    double alpha = ( it->y - (it+1)->y ) / ( it->x - (it+1)->x );
    ++it;
    while( (it+1) != this->processedPath.end( ) ) {
      double newAlpha = ( it->y - (it+1)->y ) / ( it->x - (it+1)->x );
        if ( std::fabs( newAlpha - alpha ) > tolerance ) {
            smoothPath.push_back( *it );
            alpha = newAlpha;
        } // if
        ++it;
    } // while
    smoothPath.push_back( *it );

    logger().log(opencog::Logger::INFO, "HPASearch - smoothing path complete. (%d) points.",
        smoothPath.size( ) );

    this->processedPath = smoothPath;
  }

  void HPASearch::Level::smoothTriangulate( void ) {
    if ( this->processedPath.size( ) < 3 ) {
      return;
    } // if
    
    std::vector<Math::Vector2> smoothPath;
    

    unsigned int i = 0;
    unsigned int j;
    smoothPath.push_back( this->processedPath[ 0 ] );
    while( i < this->processedPath.size( ) ) {
      bool keepTrying = true;
      for( j = i+2; keepTrying && j < this->processedPath.size( ); ++j ) {

	Spatial::GridPoint startPoint( this->map->snap( Spatial::Point
           ( this->processedPath[ i ].x, this->processedPath[ i ].y ) ) );

	Spatial::GridPoint endPoint( this->map->snap( Spatial::Point
           ( this->processedPath[ j ].x, this->processedPath[ j ].y ) ) );

	Spatial::GridPoint collision;
	bool collided = false;
	rayTrace( startPoint, endPoint, CollisionDetector( this->map, collision, collided ) );

	if ( collided ) { 
	  // there are obstacles between points
	  smoothPath.push_back( this->processedPath[ j-1 ] );
	  keepTrying = false;
	  i = j-1;
	} // else
      } // for
      i += ( keepTrying ) ? 1 : 0;
    } // for
    smoothPath.push_back( this->processedPath.back( ) );

    logger().log(opencog::Logger::INFO, "HPASearch - smoothing triangulation complete. (%d) points.", smoothPath.size( ) );

    this->processedPath = smoothPath;

  }

  //**************************************************************************//
  HPASearch::HPASearch( LocalSpaceMap2D* map, unsigned int numberOfLevels, unsigned int maximumClusters ) : map( map ), numberOfLevels( numberOfLevels ) {
    logger().log(opencog::Logger::DEBUG, "HPASearch - start building clusters");
    unsigned int i;
    for( i = 1; i <= numberOfLevels; ++i ) {
      this->levels.push_back( new Level( map, i, maximumClusters ) );
    } // for
    logger().log(opencog::Logger::DEBUG, "HPASearch - end building clusters");	

  }  

  HPASearch::~HPASearch(void) {
    unsigned int i;
    for( i = 0; i < this->levels.size( ); ++i ) {
      delete this->levels[ i ];
      this->levels[ i ] = 0;
    } // for
  }
  HPASearch::Level* HPASearch::getLevel( unsigned int level ) const throw( opencog::RuntimeException ) {
    if ( level <= this->levels.size( ) ) {
      return this->levels[ level - 1 ];
    } // if
    throw opencog::RuntimeException( TRACE_INFO, "Invalid level number" );
  }
  
  unsigned int HPASearch::getNumberOfLevels( void ) {
    return this->numberOfLevels;
  }
  
  bool HPASearch::processPath( const Math::Vector2& startPoint, const Math::Vector2& endPoint, unsigned int levelId ) throw( opencog::RuntimeException ) {
    bool response = getLevel( levelId )->processPath( startPoint, endPoint );
    logger().log(opencog::Logger::INFO, ( "HPASearch - from[" + startPoint.toString( ) + "] to[" + endPoint.toString( ) + "] Result (%d)" ).c_str( ), response ? 1 : 0 );
    return response;
  }

  const std::vector<Math::Vector2>& HPASearch::getProcessedPath( unsigned int levelId ) const throw( opencog::RuntimeException ) {
    return getLevel( levelId )->getProcessedPath( );
  }

}; // Spatial
