#ifndef INSIGHT__H
#define INSIGHT__H

#include <cache_generator.h>
#include <rubik.h>

template< size_t N >
class Insight
{
  using _crot = CRotations< 2 * N - 3 >;

  CacheID       m_stateID;
  CubeID        m_prior;

  const size_t  m_size;
  const PosID * m_pos;

  const CacheIDmap<N> * m_map;

public:
  Insight( SubSpace P, const CubeID cid = 0 );
  ~Insight();

  void set( const Rubik<N> & );

  int rotate( Axis axis, Layer layer, Turn turn );

  void step( const size_t id )
  {
    rotate( m_map -> router( m_stateID, id ) );
  }
  
  CacheID state() const
  {
    return m_stateID;
  }

  int distance() const
  {
    return m_map -> distance( m_stateID );
  }

  void print() const;
};

template< size_t N >
Insight<N>::Insight( SubSpace P, const CubeID cid )
  : m_size  ( P.size() )
{
  CacheIDmapper<N> mapBuilder;
  CacheIDmap<N>    * map = new CacheIDmap<N>();

  PosID * pos = new PosID [ P.size() ];
  size_t i = 0;
  for( auto p: P )
  {
    pos[ i++ ] = CPositions<N>::GetPosID( p, cid );
  }
  mapBuilder.initialPosition( pos, P.size() );
  mapBuilder.createMap( *map );
  m_map = map;
  m_pos = pos;
}

template< size_t N >
void Insight<N>::print() const
 {
   PosID * pos = new PosID [ m_size ];
   SetCacheID( pos, m_stateID, m_size, m_prior ); 
   for ( size_t i = 0; i < m_size; ++i )
   {
     clog( CPositions<N>::GetCoord( m_pos[i] ).toString() ,"-->", CPositions<N>::GetCoord( CPositions<N>::GetPosID( m_pos[i], pos[i] ) ).toString() );
   }
   clog( "Order: ", distance(), "Prior: ", Simplex::GetCube( m_prior ).toString() );

   static constexpr Orient _side[ 3 ][ 4 ] = { 
          { _NF,  _U, _NF, _NF },
          {  _L,  _F,  _R,  _B },
          { _NF,  _D, _NF, _NF }
   };

   //auto facet = []( const Orient o )  {  };
   for( size_t raw = 0; raw < 3 * N; ++ raw )
   {
     if ( raw % N == 0 )
       NL();

     for( size_t col = 0; col < 4 * N; ++ col )
     {
       if ( col % N == 0 )
         clog_ ( ' ' );

       const Orient side = _side[ raw / N ][ col / N ];
       bool grayFacet = true;
       for ( size_t pid = 0; pid < m_size; ++ pid )
       {
          Layer xCoord;
          Layer yCoord;

          switch( side )
          {
          case _U:
            if ( CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Y ) < N - 1 )
              continue;
            xCoord = CPositions<N>::GetLayer( m_pos[pid], pos[pid], _X );
            yCoord = CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Z );
            break;

          case _L:
            if ( CPositions<N>::GetLayer( m_pos[pid], pos[pid], _X ) > 0 )
              continue;
            xCoord =         CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Z );
            yCoord = N - 1 - CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Y );
            break;
            
          case _F:
            if ( CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Z ) < N - 1 )
              continue;
            xCoord =         CPositions<N>::GetLayer( m_pos[pid], pos[pid], _X );
            yCoord = N - 1 - CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Y );
            break;
              
          case _R:
            if ( CPositions<N>::GetLayer( m_pos[pid], pos[pid], _X ) < N - 1 )
              continue;
            xCoord = N - 1 - CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Z );
            yCoord = N - 1 - CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Y );
            break;

          case _B:
            if ( CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Z ) > 0 )
              continue;
            xCoord = N - 1 - CPositions<N>::GetLayer( m_pos[pid], pos[pid], _X );
            yCoord = N - 1 - CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Y );
            break;
          
          case _D:
            if ( CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Y ) > 0 )
              continue;
            xCoord =         CPositions<N>::GetLayer( m_pos[pid], pos[pid], _X );
            yCoord = N - 1 - CPositions<N>::GetLayer( m_pos[pid], pos[pid], _Z );
            break;

          default:
            clog_( " " );
            pid = m_size; xCoord = N; grayFacet = false; // outer break
            break;
         }

         if ( xCoord == col % N && yCoord == raw % N )
         {
           clog_( colorOf( Simplex::GetCube( Simplex::Composition( pos[pid], m_prior ) ).whatIs( side ) ), "*", Color::off );
           grayFacet = false;
           break;
         }
       }
       if ( grayFacet )
       {
         clog_( Color::dark,".", Color::off );
       }
     }
     NL();
   }
 }

template< size_t N >
void Insight<N>::set( const Rubik<N> & R )
{
  m_prior = R.getCubeID( m_pos[0] );
  CubeID * subset = new CubeID [ m_size ];
  for( size_t posIndex = 0; posIndex < m_size; ++ posIndex )
  {
    subset[ posIndex ] = R.getCubeID( m_pos[ posIndex ] );
  }
  m_stateID = GetCacheID( subset, m_size );
  delete[] subset;
}

template< size_t N >
int Insight<N>::rotate( Axis axis, Layer layer, Turn turn )
{
  clog( "state:", m_stateID );
  clog_( _crot::ToString( CExtRotations<N>::GetRotID( axis, layer, turn ) ), "-->" );
  const RotID rotID = CExtRotations<N>::GetRotID( axis, layer, turn, Simplex::Inverse( m_prior ) );
  clog( _crot::ToString( rotID ) );
  if ( ( layer  < N && layer         == CPositions<N>::GetLayer( m_pos[0], m_prior, axis ) ) ||
       ( layer >= N && layer - N + 1 >= CPositions<N>::GetLayer( m_pos[0], m_prior, axis ) ) ) 
  {
    clog_( Simplex::GetCube( m_prior).toString(), "-->" );
    m_prior = Simplex::Tilt( m_prior, axis, turn ); clog( Simplex::GetCube( m_prior).toString() );
  }
    
  m_stateID = m_map -> getState( m_stateID, rotID ) ;
  print();

  return distance();
}

template< size_t N >
Insight<N>::~Insight()
{
  delete   m_map;
  delete[] m_pos;
}
#endif //  ! INSIGHT__H
