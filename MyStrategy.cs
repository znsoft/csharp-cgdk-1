using System;
using System.Collections;
using System.Linq;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.Model;
using System.Collections.Generic;

namespace Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk {
    public sealed class MyStrategy : IStrategy {
        const int MAXSTOPCOUNT = 30;
        const double MINSPEED = 0.09D;
        const int MAXBACKTICKS = 190;
        const int MAXWAYITERATIONS = 100;//максимально возможна€ длина пути (ограничить проц врем€)
        const int MAXERRORBLOCKS = 8;
        const double PRETURNSPEEDMUL = 15.5D;
        const double CORNERCORRECTION = -0.25D;
        const double FORWARDWALLDETECT = 15.0D;
        enum MovingState
        {
            FORWARD,
            BACKWARD,
            STOP
        }

        MovingState currentState = MovingState.FORWARD;
        int stateTickCount = 0;


        Dictionary<TileType, Direction[]> fromTile = new Dictionary<TileType, Direction[]>();

        Car self;
        World world;
        Game game;
        Move move;
        double speedModule;
        private int stopTickCount = 0;
       
        MyMap[][] myMap;
        //int errorBlockCount = 0; //если столкнулись со стеной то несколько блоков едем "осторожно" без предсказаний

        public MyStrategy() {
            FillFromTileTable();
        }


        public void Move(Car self, World world, Game game, Move move)
        {
           
            move.EnginePower = 1.0D;// (0.95D);
            Construct(self, world, game, move);
            AnalyzeCurrentSpeedAndState();
            MyWay way = FindWay(self.X,self.Y, self.NextWaypointX, self.NextWaypointY);
            double nextWaypointX = way.GetCenterX(game.TrackTileSize);
            double nextWaypointY = way.GetCenterY(game.TrackTileSize);
            DebugNextWay(way.x2, way.y2);
            double distance = self.GetDistanceTo(InvTransoform(self.NextWaypointX), InvTransoform(self.NextWaypointY));
            //errorBlockCount = 1;
            if (distance < (game.TrackTileSize + speedModule * PRETURNSPEEDMUL) )//800*1250*32/
            {
                int nextWayPointIndex = (self.NextWaypointIndex + 1) % world.Waypoints.Length;
                way = FindWay(self.X, self.Y, world.Waypoints[nextWayPointIndex][0], world.Waypoints[nextWayPointIndex][1]);
                nextWaypointX = way.GetCenterX(game.TrackTileSize);
                nextWaypointY = way.GetCenterY(game.TrackTileSize);
                move.IsSpillOil = true;
            }
            else
            {
                // errorBlockCount = 0;
                if (world.Tick > game.InitialFreezeDurationTicks)
                    if (distance > (game.TrackTileSize * 5) && isNoWallAtLine(self.X, self.Y, new MyWay(self.NextWaypointX, self.NextWaypointY), 10)) 
                move.IsUseNitro = true;
            }
            DebugNextWay(way.x2, way.y2);
            //  if (errorBlockCount > 0) move.EnginePower = 0.75D; else
            CorrectCenterPoint(ref nextWaypointX, ref nextWaypointY);

            if (distance < game.TrackTileSize * speedModule)//800*1250*32/
            {
                move.EnginePower = 15.0D / speedModule;
                //if (errorBlockCount > 0) move.IsBrake = true;
            }
            if (distance > game.TrackTileSize * 15)//800*1250*32/
            {
                //move.IsUseNitro = true;
            }
            double angleToWaypoint = self.GetAngleTo(nextWaypointX, nextWaypointY);
            move.WheelTurn = angleToWaypoint * 32.0D / Math.PI;
            if (speedModule * speedModule * Math.Abs(angleToWaypoint) > 2.5D * 2.5D * Math.PI )
            {
                //move.IsBrake = true;
                //move.IsUseNitro = true;
            }

            if (currentState == MovingState.BACKWARD)
            {
                if(stateTickCount < MAXBACKTICKS /2)
                    move.EnginePower = -1;
                //move.IsUseNitro = true;
                move.IsBrake = false;
                move.WheelTurn = -move.WheelTurn;
            }


            FireinEnemy(move);


        }

        private bool IsNearWallsEdges(double forwardWallDetect)
        {
            return false;
            double fx = self.SpeedX * forwardWallDetect;
            double fy = self.SpeedY * forwardWallDetect;

            int myX = Transform(self.X);
            int myY = Transform(self.Y);
            double cellX = self.X - DTransoform0(myX);
            double cellY = self.Y - DTransoform0(myY);
            double p = game.TrackTileSize / 25 + game.CarWidth / 2;
            if (OnMyLine(cellX, cellY, 0, 0, fx, fy, p) ||
                            OnMyLine(cellX, cellY, game.TrackTileSize, 0, fx, fy, p) ||
                            OnMyLine(cellX, cellY, 0, game.TrackTileSize, fx, fy, p) ||
                            OnMyLine(cellX, cellY, game.TrackTileSize, game.TrackTileSize, fx, fy, p))
            {  return true; }
            return false;
                
        }


        private bool OnMyLine(double px,double py, double x1, double y1, double ex, double ey, double p)
        {

            double dx = ex - px;
            double dy = ey - py;
            double x = x1;
            double y = y1;
                if (Math.Abs(dx) > Math.Abs(dy))
                {
                    y = py + (x - px) * (dy) / (dx);
                }
                else
                {
                    x = px + (y - py) * (dx) / (dy);
                }

            double h = Hypot(x1 - x, y1 - y);
            return h < p;
        }

        private bool IsBackSpeed(double speedModule)
        {
            return Math.Abs(Math.Cos(self.Angle) * speedModule - self.SpeedX) +
           Math.Abs(Math.Sin(self.Angle) * speedModule - self.SpeedY) < 0.1D;
        }


        private void DebugSpeed(double speedModule)
        {
            Console.WriteLine((Math.Cos(self.Angle) * speedModule).ToString() + " " + self.SpeedX.ToString() + " " + (Math.Sin(self.Angle) * speedModule).ToString() + " " + self.SpeedY.ToString());

        }

        private void FireinEnemy(Move move)
        {
            if (WhoOnMyFireLine() != null) move.IsThrowProjectile = true;
        }

        private void CorrectCenterPoint(ref double nextWaypointX, ref double nextWaypointY)
        {
            double cornerTileOffset = CORNERCORRECTION * game.TrackTileSize;

            switch (world.TilesXY[Transform(nextWaypointX)][Transform(nextWaypointY)])
            {
                case TileType.LeftTopCorner:
                    nextWaypointX += cornerTileOffset;
                    nextWaypointY += cornerTileOffset;
                    break;
                case TileType.RightTopCorner:
                    nextWaypointX -= cornerTileOffset;
                    nextWaypointY += cornerTileOffset;
                    break;
                case TileType.LeftBottomCorner:
                    nextWaypointX += cornerTileOffset;
                    nextWaypointY -= cornerTileOffset;
                    break;
                case TileType.RightBottomCorner:
                    nextWaypointX -= cornerTileOffset;
                    nextWaypointY -= cornerTileOffset;
                    break;
                default:
                    break;
            }
        }

        private  void DebugMap()
        {
            for (int x = 0; x < world.TilesXY.Length; x++)
            {
                for (int y = 0; y < world.TilesXY[x].Length; y++)
                {
                    Console.Write((int)world.TilesXY[x][y]);
                }
                Console.WriteLine(" ");

            }
            Console.WriteLine("-------");

            for (int x = 0; x < world.Waypoints.Length; x++)
            {
                for (int y = 0; y < world.Waypoints[x].Length; y++)
                {
                    Console.Write((int)world.Waypoints[x][y]); Console.Write(" ");
                }
                Console.WriteLine(" ");

            }
            //DebugNextWay();
        }


        private void DebugFindMap()
        {
            for (int x = 0; x < myMap.Length; x++) { 
                for (int y = 0; y < myMap[x].Length; y++)
                {
                    MyMap myTile = myMap[x][y];
                    Console.Write(myTile.lenCount);
                }
                Console.WriteLine(" ");

            }
            Console.WriteLine("-------");
        }

        private void DebugNextWay(int x,int y)
        {
            Console.WriteLine(x.ToString() + " " + y.ToString());
            Console.WriteLine("-------");
        }

        MyWay FindWay(double px,double py, int wx, int wy)
        {
            CopyMap();
            int x = Transform(px);
            int y = Transform(py);
            myMap[x][y].lenCount = 1;// startpoint
            int lenCount = FillShortWay(wx, wy);
           // DebugFindMap();
            //DebugNextWay(wx, wy);
            //Console.WriteLine("---my----");
           // DebugNextWay(x, y);
            if (lenCount <= 2) return new MyWay(wx, wy);
            MyWay[] myWay = new MyWay[lenCount];
            myWay[lenCount - 1] = new MyWay(wx, wy, myMap[wx][wy]);
            if (isNoWallAtLine(px, py, myWay[lenCount - 1], lenCount)) {
                return myWay[lenCount - 1]; }
            bool isNearWall = IsNearWallsEdges(FORWARDWALLDETECT);
            for (int i = lenCount-1; i > 1; i--) {
                //double accelerate = (lenCount - i) * (1.0D / lenCount);
                myWay[i-1] = FindAround(i, myWay );
                //myWay[i - 1].Acelerate = accelerate;
                if (isNearWall) continue;
                if ( isNoWallAtLine(px, py, myWay[i - 1], i))return myWay[i - 1];
            }

             return myWay[1];
        }

 

        bool isNoWallAtLine(double x0, double y0, MyWay myWay, int lenCount) {
            double x1 = myWay.GetCenterX(game.TrackTileSize);
            double y1 = myWay.GetCenterY(game.TrackTileSize);
            double x = x0, y = y0;
            double dx = x1 - x0;
            double dy = y1 - y0;
            int xw, yw;

            int i=0, l;

            while (true) {
            if (Math.Abs(dx) > Math.Abs(dy)) {
                    l = (int)Math.Abs(dx);
                    y = y0 + (x - x0) * (dy) / (dx);
                    x += dx > 0 ? 1.0D : -1.0D; 
                } else {
                    l = (int)Math.Abs(dy);
                    x = x0 + (y - y0) * (dx) / (dy);
                    y += dy > 0 ? 1.0D : -1.0D;
                }
                if (++i >= l) break;
                xw = Transform(x);
                yw = Transform(y);
                MyMap myTile = new MyMap();
                int w = WaveAt(xw, yw, myTile);
                if (w == 0 || w > lenCount)  return false;
                if (IsInnerWallIn(x-DTransoform0(xw) , y-DTransoform0(yw), myTile)) return false;
        }
            return true;

        }

        private bool IsInnerWallIn(double v1, double v2, MyMap myTile)
        {

            if (CoordsInEdgeRadius(v1, v2, game.TrackTileSize / 10 )) return true;//game.CarWidth/1.9D + 
                                                                                 // if (!fromTile.ContainsKey(myTile.tile)) return false;
                                                                                 //Direction[] dir = fromTile[myTile.tile];
            return false;
        }

        private bool CoordsInEdgeRadius(double v1, double v2,double radius)
        {
            if (Hypot(v1, v2) < radius) return true;
            if (Hypot(game.TrackTileSize-v1, game.TrackTileSize-v2) < radius) return true;
            if (Hypot(v1, game.TrackTileSize - v2) < radius) return true;
            if (Hypot(game.TrackTileSize - v1,  v2) < radius) return true;
            return false;
        }

        MyWay FindAround(int i, MyWay[] tempWay)
        {
            int  x = tempWay[i].x2;
            int  y = tempWay[i].y2;
            //bug необходимо учитывать тип тайла иначе может возникнуть ситуаци€ 
            //    123 - от этой двойки никак не пробратьс€ к нижней тройке
            //    23
            Direction[] dir = new Direction[] { };

            MyMap thisTile = tempWay[i].myTile;
            if (fromTile.ContainsKey(thisTile.tile))
                dir = fromTile[thisTile.tile];

            MyWay myWay = DirectionsContainsWave(i, x,  y, dir);
            //if(x==x1&&y==y1)//баг если к этой точке придут 2 пути одинаковой длины то алгоритм зависнет
            MyMap myTile = new MyMap();
            WaveAt(myWay.x2, myWay.y2, myTile);
            myWay.myTile = myTile;
            return myWay;
        }

        private MyWay DirectionsContainsWave(int i,  int x, int y, Direction[] dir)
        {
            MyWay myWay = new MyWay(x, y);
            if (dir.Contains(Direction.Right) && isNextWaveAt(x + 1, y, i)) { myWay.x2 = x + 1; return myWay; }
            if (dir.Contains(Direction.Left) && isNextWaveAt(x - 1, y, i)) { myWay.x2 = x - 1; return myWay; }
            if (dir.Contains(Direction.Down) && isNextWaveAt(x, y + 1, i)) { myWay.y2 = y + 1; return myWay; }
            if (dir.Contains(Direction.Up) && isNextWaveAt(x, y - 1, i)) { myWay.y2 = y - 1; return myWay; }
            return myWay;
        }

        private bool isNextWaveAt(int x, int y, int i)
        {
            return WaveAt( x, y) == i;
        }

        int WaveAt(int x, int y) {
            MyMap myTile = new MyMap();// = myMap[x][y];
            return WaveAt(x, y, myTile);
        }


        int WaveAt(int x, int y, MyMap myTile)
        {
            if (x < 0 || y < 0 || x >= myMap.Length || y >= myMap[x].Length) return 0;
            myTile.tile = myMap[x][y].tile;
            myTile.lenCount = myMap[x][y].lenCount;
            if (!fromTile.ContainsKey(myTile.tile)) return 0;
            return myTile.lenCount;
        }


        private int FillShortWay(int nextWaypointX, int nextWaypointY)
        {
            int shortLen;
            for (int i = MAXWAYITERATIONS; i >= 0; i--)
            {
                shortLen = FillOneWave(nextWaypointX, nextWaypointY);
                if (shortLen > 0) return shortLen;
            }
            return 0;
        }

        private void FillFromTileTable()
        {
            fromTile.Add(TileType.Vertical, new Direction[] { Direction.Down, Direction.Up });
            fromTile.Add(TileType.Horizontal, new Direction[] { Direction.Left, Direction.Right });
            fromTile.Add(TileType.Crossroads, new Direction[] { Direction.Left, Direction.Right, Direction.Down, Direction.Up });
            fromTile.Add(TileType.BottomHeadedT, new Direction[] { Direction.Left, Direction.Right, Direction.Down });
            fromTile.Add(TileType.LeftBottomCorner, new Direction[] { Direction.Right, Direction.Up });
            fromTile.Add(TileType.LeftHeadedT, new Direction[] { Direction.Left, Direction.Up, Direction.Down });
            fromTile.Add(TileType.LeftTopCorner, new Direction[] { Direction.Right, Direction.Down });
            fromTile.Add(TileType.RightBottomCorner, new Direction[] { Direction.Left, Direction.Up });
            fromTile.Add(TileType.RightHeadedT, new Direction[] { Direction.Right, Direction.Down, Direction.Up });
            fromTile.Add(TileType.RightTopCorner, new Direction[] { Direction.Left, Direction.Down });
            fromTile.Add(TileType.TopHeadedT, new Direction[] { Direction.Left, Direction.Right, Direction.Up });
        }

        int FillOneWave(int wx,int wy)
        {
            for (int x = 0; x < myMap.Length; x++)
                for (int y = 0; y < myMap[x].Length; y++)
                {
                    MyMap myTile = myMap[x][y];
                    if (myTile.lenCount == 0) continue;
                    if (myTile.isFillAround) continue;
                    if (myTile.isWall()) continue;
                    if (x == wx && y == wy) return myTile.lenCount;
                    FillAround(x, y, myTile);
                    myMap[x][y] = myTile;
            }
            return 0;
        }

        private void FillAround(int x, int y, MyMap myTile)
        {
            if (!fromTile.ContainsKey(myTile.tile))return;
            Direction[] openDirs = fromTile[myTile.tile];
            foreach (Direction dir in openDirs) {
                FillOpenDirection(x, y, dir, myTile.lenCount + 1 );
            }
            myTile.isFillAround = true;

        }

        private bool FillOpenDirection(int x, int y, Direction dir, int lenCount)
        {
            switch (dir)
            {
                case Direction.Down:
                    return FillDirection(x, y + 1, dir, lenCount);
                case Direction.Up:
                    return FillDirection(x, y - 1, dir, lenCount);
                case Direction.Right:
                    return FillDirection(x + 1, y, dir, lenCount);
                case Direction.Left:
                    return FillDirection(x - 1, y, dir, lenCount);
            }
            return false;
        }

        private bool FillDirection(int x, int y, Direction dir, int lenCount)
        {
            if (x < 0 || y < 0 || x >= myMap.Length || y >= myMap[x].Length) return false;
            MyMap myTile = myMap[x][y];
            if (!fromTile.ContainsKey(myTile.tile)) return false;
            if (myTile.lenCount > 0) return false;
            myTile.lenCount = lenCount;
            myMap[x][y] = myTile;
            return true;

        }

        void CopyMap()
        {
            myMap = new MyMap[world.TilesXY.Length][];
            for (int x = 0; x < myMap.Length; x++)
            {
                myMap[x] = new MyMap[world.TilesXY[x].Length];
                for (int y = 0; y < myMap[x].Length; y++)
                {
                    MyMap myTile = new MyMap();
                    myTile.tile = world.TilesXY[x][y];
                    myTile.isFillAround = false;
                    myTile.lenCount = 0;
                    myMap[x][y] = myTile;
                }
            }
        }

        private void AnalyzeCurrentSpeedAndState()
        {
            
            speedModule = Hypot(self.SpeedX, self.SpeedY);
            if (speedModule < MINSPEED && currentState == MovingState.FORWARD && (world.Tick > game.InitialFreezeDurationTicks))           
                stopTickCount++;
            else
                stopTickCount = 0;
            if (stopTickCount > MAXSTOPCOUNT) { 
                currentState = MovingState.BACKWARD;
                stopTickCount = 0;
                stateTickCount = 0;
                if (WallCollisionDetect())
                {
                    //errorBlockCount = MAXERRORBLOCKS;
                }
            }
            if (currentState == MovingState.BACKWARD) {
                stateTickCount++;
                if (stateTickCount > MAXBACKTICKS) {
                    currentState = MovingState.FORWARD;
                }
            }

        }

        private bool WallCollisionDetect()
        {
            double f = Hypot(game.CarHeight, game.CarWidth) + game.TrackTileSize / 10.0D;
            double fx = self.X + f * Math.Cos(self.Angle);
            double fy = self.Y + f * Math.Sin(self.Angle);
            return Transform(fx) != Transform(self.X) || Transform(fy) != Transform(self.Y);
        }

        private void Construct(Car self, World world, Game game, Move move)
        {
            this.self = self;
            this.world = world;
            this.move = move;
            this.game = game;
        }



        private Car WhoOnMyFireLine()
        {

            var search = from Car enemy in world.Cars
                         where
                         IsEnemyOnMyFireLine(enemy)
                         select enemy;
            return search.FirstOrDefault();

        }

        private bool IsEnemyOnMyFireLine(Car enemy)
        {
            return !enemy.IsTeammate && Math.Abs(self.GetAngleTo(enemy)) < 0.1D
                                     && self.GetDistanceTo(enemy) < game.TrackTileSize*2;
        }



        double Hypot(double x, double y) { return Math.Sqrt(x * x + y * y); }
        int Transform(double x) { return (int)( (x ) / game.TrackTileSize); }
        double InvTransoform(int x) { return (double)(x + 0.5D) * game.TrackTileSize; }
        double DTransoform0(int x) { return (double)(x ) * game.TrackTileSize; }


        class MyWay
        {
            public int x1, y1, x2, y2;
            public double Acelerate;
            public MyMap myTile;

            public MyWay()
            {
            }

            public MyWay(int x, int y) {
                x2 = x; y2 = y;
            }

            public MyWay(int x, int y, MyMap myTile)
            {
                x2 = x; y2 = y;this.myTile = myTile;
            }

            public double GetCenterX(double TrackTileSize) {
                return (double) (x2 + 0.5D) * TrackTileSize;
            }

            public double GetCenterY(double TrackTileSize)
            {
                return (double)(y2 + 0.5D) * TrackTileSize;
            }

        }

        public class MyMap
        {
            public TileType tile;
            public int lenCount;
            public bool isFillAround;
            public bool isWall() {
                return tile == TileType.Empty;
            }
        }
    }


}
