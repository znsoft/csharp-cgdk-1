using System;
using System.Collections;
using System.Linq;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.Model;
using System.Collections.Generic;

namespace Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk {
    public sealed class MyStrategy : IStrategy {
        const int MAXSTOPCOUNT = 20;
        const double MINSPEED = 0.08D;
        const int MAXBACKTICKS = 80;
        const int MAXWAYITERATIONS = 100;


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

        public MyStrategy() {
            FillFromTileTable();
        }


        public void Move(Car self, World world, Game game, Move move)
        {
            Construct(self, world, game, move);
            AnalyzeCurrentSpeedAndState();
            //if (world.Tick < game.InitialFreezeDurationTicks) FindWays();

            move.IsThrowProjectile = true;
            move.IsSpillOil = true;

            if (world.Tick > game.InitialFreezeDurationTicks)
            {
               // move.IsUseNitro = true;
            }

            // DebugMap(self, world);
            MyWay way = FindWay();
            double nextWaypointX = way.GetCenterX(game.TrackTileSize);
            double nextWaypointY = way.GetCenterY(game.TrackTileSize);
            Console.WriteLine("-------");
            Console.Write(TransformX(nextWaypointX));
            Console.Write(" ");
            Console.WriteLine(self.NextWaypointX);

            Console.Write(TransformY(nextWaypointY));
            Console.Write(" ");
            Console.WriteLine(self.NextWaypointY);
            Console.WriteLine(self.NextWaypointIndex);
            Console.WriteLine("-------");
            //CorrectCenterPoint(self, world, game, ref nextWaypointX, ref nextWaypointY);

            double angleToWaypoint = self.GetAngleTo(nextWaypointX, nextWaypointY);
            Console.WriteLine(angleToWaypoint);


            move.WheelTurn = angleToWaypoint * 32.0D / Math.PI;
            move.EnginePower = (0.95D);

            if (speedModule * speedModule * Math.Abs(angleToWaypoint) > 2.5D * 2.5D * Math.PI)
            {
                move.IsBrake = true;
                //move.IsUseNitro = true;
            }

            if (currentState == MovingState.BACKWARD)
            {
                move.EnginePower = -1;
                move.IsUseNitro = true;
                move.IsBrake = false;
                move.WheelTurn = -move.WheelTurn;
            }





        }

        private static void CorrectCenterPoint(Car self, World world, Game game, ref double nextWaypointX, ref double nextWaypointY)
        {
            double cornerTileOffset = 0.25D * game.TrackTileSize;

            switch (world.TilesXY[self.NextWaypointX][self.NextWaypointY])
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

        private static void DebugMap(Car self, World world)
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
            Console.WriteLine("-------");
            Console.WriteLine(self.NextWaypointX);
            Console.WriteLine(self.NextWaypointY);
            Console.WriteLine(self.NextWaypointIndex);
            Console.WriteLine("-------");
        }

        MyWay FindWay()
        {
            CopyMap();
            int x = TransformX(self.X);// / game.TrackTileSize);
            int y = TransformY(self.Y);// / game.TrackTileSize);
            myMap[x][y].lenCount = 1;// startpoint
            int wx = self.NextWaypointX;
            int wy = self.NextWaypointY;
            int lenCount = FillShortWay(wx, wy);
            //List<MyWay> myWay = new List<MyWay>();
            if (lenCount <= 2) return new MyWay(wx, wy);
            MyWay[] myWay = new MyWay[lenCount];
            myWay[lenCount - 1] = new MyWay(wx, wy);
            if (isAnyWallAtLine(self.X, self.Y, myWay[lenCount - 1], lenCount)) {
                Console.WriteLine("-");
                return myWay[lenCount - 1]; }
            for (int i = lenCount-1; i >= 1; i--) {
                double accelerate = (lenCount - i) * (1.0D / lenCount);
                myWay[i-1] = FindAround(i, myWay );
                myWay[i - 1].Acelerate = accelerate;
                if (isAnyWallAtLine(self.X, self.Y, myWay[i - 1], i))return myWay[i - 1];
            }

            return myWay[1];
        }

        int TransformX(double x) { return (int)(x / game.TrackTileSize); }
        int TransformY(double y) { return (int)(y / game.TrackTileSize); }

        bool isAnyWallAtLine(double x, double y, MyWay myWay, int lenCount) {
            double x2 = myWay.GetCenterX(game.TrackTileSize);
            double y2 = myWay.GetCenterX(game.TrackTileSize);
            double dx = x2 - x;
            double dy = y2 - y;
            double d;
            int i=0, l;

            while (true) { 
            if (Math.Abs(dx) > Math.Abs(dy)) { l = (int)Math.Abs(dx); d = dx == 0 ? 0 : dy / Math.Abs(dx); x += 1.0D; y += d; } else { l = (int)Math.Abs(dy); d = dy == 0 ? 0 : dx / Math.Abs(dy); y += 1.0D; x += d; }
            if (++i >= l) break;
                int w = WaveAt(TransformX(x), TransformY(y));
                if (w == 0 || w > lenCount) return false;
        }
            return true;

        }



        MyWay FindAround(int i, MyWay[] tempWay)
        {
            int x1,x = tempWay[i].x2;
            int y1,y = tempWay[i].y2;
            x1 = x;         y1 = y;
            if (isNextWaveAt(x + 1, y, i)) x1 = x + 1;
            if (isNextWaveAt(x - 1, y, i)) x1 = x - 1;
            if (isNextWaveAt(x , y+1, i)) y1 = y + 1;
            if (isNextWaveAt(x , y-1, i)) y1 = y - 1;
            MyMap myTile = new MyMap();
            WaveAt(x1, y1, myTile);
            return new MyWay(x1,y1, myTile);
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
            myTile = myMap[x][y];
            if (!fromTile.ContainsKey(myTile.tile)) return 0;
            return myTile.lenCount;
        }


        private int FillShortWay(int nextWaypointX, int nextWaypointY)
        {
            int shortLen;
            for (int i = MAXWAYITERATIONS; i >= 0; i--)
            {
                shortLen = FillOneWave(self.NextWaypointX, self.NextWaypointY);
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
            }
            if (currentState == MovingState.BACKWARD) {
                stateTickCount++;
                if (stateTickCount > MAXBACKTICKS) {
                    currentState = MovingState.FORWARD;
                }
            }

        }

        private void Construct(Car self, World world, Game game, Move move)
        {
            this.self = self;
            this.world = world;
            this.move = move;
            this.game = game;
        }

        double Hypot(double x, double y) { return Math.Sqrt(x * x + y * y); }

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
