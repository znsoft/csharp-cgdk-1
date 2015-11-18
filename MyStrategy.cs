using System;
using System.Collections;
using System.Linq;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.Model;
using System.Collections.Generic;

namespace Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk {
    public sealed class MyStrategy : IStrategy {
        const int MAXSTOPCOUNT = 20;
        const double MINSPEED = 0.08D;
        const int MAXBACKTICKS = 100;

        enum MovingState
        {
            FORWARD,
            BACKWARD
        }

        MovingState currentState = MovingState.FORWARD;
        int stateTickCount = 0;
        
        Car self;
        World world;
        Game game;
        Move move;
        double speedModule;
        private int stopTickCount = 0;
        List<MyWay> myWay;
        MyMap[][] myMap;

        public void Move(Car self, World world, Game game, Move move) {
            Construct(self, world, game, move);
            AnalyzeCurrentSpeedAndState();
            if (world.Tick < game.InitialFreezeDurationTicks) FindWays();

            
            move.IsThrowProjectile = true;
            move.IsSpillOil = true;

            if (world.Tick > game.InitialFreezeDurationTicks) {
                move.IsUseNitro = true;
            }

            for (int x = 0; x < world.TilesXY.Length ; x++)
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

/*
            -------
00315000
03604500
36003715
A1116002
45000002
04500002
00481186
00041160
------ -
2 6 = 0
3 0 = 1
6 3 = 2
6 6 = 3
5 7
3 7
------ -
3
0
1
------ -
*/
            double nextWaypointX = (self.NextWaypointX + 0.5D) * game.TrackTileSize;
            double nextWaypointY = (self.NextWaypointY + 0.5D) * game.TrackTileSize;

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

            double angleToWaypoint = self.GetAngleTo(nextWaypointX, nextWaypointY);
            
            
            move.WheelTurn = angleToWaypoint * 132.0D / Math.PI;
            move.EnginePower = (0.75D);

            if (speedModule * speedModule * Math.Abs(angleToWaypoint) > 2.5D * 2.5D * Math.PI)
            {
                move.IsBrake = true;
            }

            if (currentState == MovingState.BACKWARD) {
                move.EnginePower = -1;
                move.IsUseNitro = true;
                move.IsBrake = false;
                move.WheelTurn = -move.WheelTurn;
            }

            



        }

        private void FindWays()
        {
            CopyMap(); 


            //int x = (int)(self.X / game.TrackTileSize);
            //int y = (int)(self.Y / game.TrackTileSize);

            



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
                    myTile.lenCount = 0;
                    myMap[x][y] = myTile;
                }
            }
        }

    /*    bool FindOneWay(int x, int y, int toWayPoint, ) {
            
            int wx = world.Waypoints[toWayPoint][0];
            int wy = world.Waypoints[toWayPoint][1];
            if (x == wx && y == wy) return  true;
            List<MyWay> thisWay = new List<MyWay>();
                if (isRoadOpen(x,y+1))FindOneWay(x , y+1, toWayPoint)

        }*/

        private bool isRoadOpen(int x, int y, Direction dir)
        {
            switch (dir) {
                case Direction.Down:
                    return isRoadOpenDir(x, y + 1, dir);
                case Direction.Up:
                    return isRoadOpenDir(x, y - 1, dir);
                case Direction.Right:
                    return isRoadOpenDir(x+1, y, dir);
                case Direction.Left:
                    return isRoadOpenDir(x-1, y, dir);
            }
            return false;
        }

        private bool isRoadOpenDir(int x, int y, Direction dir)
        {
            if (x < 0 || y < 0 || x >= world.TilesXY.Length || y >= world.TilesXY[x].Length) return false;
            TileType t = world.TilesXY[x][y];
            if (t == TileType.Empty) return false;
            if


            return true;
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

        private class MyWay
        {
            public int x1, y1, x2, y2;
            public bool useIt;
        }

        class MyMap
        {
            public TileType tile;
            public int lenCount;
            public Direction dir;
            public bool isWall() {
                return tile == TileType.Empty;
            }
        }
    }


}
