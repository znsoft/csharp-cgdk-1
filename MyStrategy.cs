using System;
using System.Collections;
using System.Linq;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.Model;
using System.Collections.Generic;

namespace Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk
{
    public sealed class MyStrategy : IStrategy
    {
        const int MAXSTOPCOUNT = 30;
        const double MINSPEED = 0.09D;
        const int MAXBACKTICKS = 300;
        const double PRECALCDIRECTIONOFFSET = 25;
        const int MAXWAYITERATIONS = 20;//максимально возможная длина пути (ограничить проц время)
        const int MAXERRORBLOCKS = 8;
        const double PRE_TURN_SPEEDMUL = 15.5D;
        const double CORNERCORRECTION = -0.23D;
        const double FORWARDWALLDETECT = 15.0D;
        const double LINESTEP = 15.0D;
        const double BREAKTRESHHOLD = 3900.0D;
        const double FORECASTMUL = 3.0D;

        enum MovingState
        {
            FORWARD,
            BACKWARD,
            STOP
        }

        MovingState currentState = MovingState.FORWARD;
        int stateTickCount = 0;


        Dictionary<TileType, Direction[]> fromTile = new Dictionary<TileType, Direction[]>();
        Dictionary<TileType, Direction[]> wallTile = new Dictionary<TileType, Direction[]>();
        TileType[] ForwardDirectionTiles = new TileType[] { TileType.Horizontal, TileType.Vertical };

        Car self;
        World world;
        Game game;
        Move move;
        double speedModule;
        private int stopTickCount = 0;
        Vector2 myFuturePos;
        WaveMapCell[][] myMap;
        private double angleAfter3Cells;
        double angleToWaypoint;
        //int errorBlockCount = 0; //если столкнулись со стеной то несколько блоков едем "осторожно" без предсказаний

        public MyStrategy()
        {
            FillFromTileTable();
            FillWallTileTable();
        }

    

        public void Move(Car self, World world, Game game, Move move)
        {
            //Console.WriteLine(self.AngularSpeed.ToString());
            myFuturePos = new Vector2(self.X + self.SpeedX * FORECASTMUL, self.Y + self.SpeedY * FORECASTMUL);
            move.EnginePower = 1.0D;// (0.95D);
            Construct(self, world, game, move);
            AnalyzeCurrentSpeedAndState();
            MyWay way = FindOptimalWay(myFuturePos.x, myFuturePos.y, self.NextWaypointX, self.NextWaypointY);
            double distance = self.GetDistanceTo(InvTransoform(self.NextWaypointX), InvTransoform(self.NextWaypointY));
            PreCalcNextWayPoint(self, world, game, move, ref way, distance);
            if (IsForwardWallsEdges()) {

                Console.WriteLine(self.AngularSpeed.ToString());
            }
            angleToWaypoint = GetAngleFromTo(myFuturePos.x, myFuturePos.y , self.Angle, way.target.x, way.target.y);
            move.WheelTurn = angleToWaypoint * 32.0D / Math.PI;
            angleToWaypoint = self.GetAngleTo(way.target.x,way.target.y);
            if (speedModule * speedModule * speedModule * Math.Abs(angleToWaypoint) > BREAKTRESHHOLD || speedModule * Math.Abs(angleAfter3Cells)>40)
            {
                move.IsBrake = true;
            }

            BackMove(move);
            FireinEnemy(move);
            OilFireEnemy(move);

        }

        public double GetAngleFromTo(double x, double y, double fromAngle, double x1, double y1)
        {
            double absoluteAngleTo = Math.Atan2(y1 - y, x1 - x);
            double relativeAngleTo = absoluteAngleTo - fromAngle;

            while (relativeAngleTo > Math.PI)
            {
                relativeAngleTo -= 2.0D * Math.PI;
            }

            while (relativeAngleTo < -Math.PI)
            {
                relativeAngleTo += 2.0D * Math.PI;
            }

            return relativeAngleTo;
        }

        private void BackMove(Move move)
        {
            if (currentState == MovingState.BACKWARD)
            {
                if (stateTickCount < MAXBACKTICKS / 3)
                    move.EnginePower = -1;
                //move.IsUseNitro = true;
                move.IsBrake = false;
                if (stateTickCount < MAXBACKTICKS / 2) move.WheelTurn = -move.WheelTurn*1.8D;
            }
        }

        private void OilFireEnemy(Move move)
        {
            TileType[] notNeedOil = new TileType[] {TileType.Crossroads, TileType.Horizontal, TileType.Vertical };
            TileType currentTile = world.TilesXY[Transform(self.X)][Transform(self.Y)];
            //if (!notNeedOil.Contains(currentTile)||
                if(WhoOnMyFireLine(IsEnemyBehindMe) != null)
            {
                move.IsSpillOil = true;
            }
        }



        private void CorrectInOutWayPoint(MyWay way)
        {

            if (!way.isInDirected && !way.isOutDirected) return;
            if (way.target == null) way.CalcTargetCenter(game.TrackTileSize);
            double size = speedModule * PRECALCDIRECTIONOFFSET + game.TrackTileSize / 2;

            double deltaShift = 0.35 * size;
            Vector2 next = way.target;
            Vector2 offset = null;
            if (way.isInDirected) offset = GetInOffset(way.dirIn, next, size - deltaShift);
            if (offset == null && way.isOutDirected)
                offset = GetOutOffset(way.dirOut, next, size + deltaShift);
            if (offset == null) return;
            way.target.x += offset.x;
            way.target.y += offset.y;
        }

        private Vector2 GetOutOffset(Direction dir, Vector2 next, double size)
        {

            switch (dir)
            {
                case Direction.Down:
                        return new Vector2(0, size);
                case Direction.Up:
                        return new Vector2(0, -size);
                case Direction.Right:
                        return new Vector2(size, 0);
                case Direction.Left:
                        return new Vector2(-size, 0);
            }
            return null;
        }


        private Vector2 GetInOffset(Direction dir, Vector2 next, double size)
        {

            switch (dir)
            {
                case Direction.Down:
                    if ((next.y + size) > self.Y)
                        return new Vector2(0, size);
                    break;
                case Direction.Up:
                    if ((next.y - size) < self.Y)
                        return new Vector2(0, -size);
                    break;
                case Direction.Right:
                    if ((next.x + size) > self.X)
                        return new Vector2(size, 0);
                    break;
                case Direction.Left:
                    if ((next.x - size) < self.X)
                        return new Vector2(-size, 0);
                    break;
            }
            return null;
        }

        private void PreCalcNextWayPoint(Car self, World world, Game game, Move move, ref MyWay way, double distance)
        {


            if (self.GetDistanceTo(InvTransoform(self.NextWaypointX), InvTransoform(self.NextWaypointY)) < (game.TrackTileSize + speedModule * PRE_TURN_SPEEDMUL))//800*1250*32/
            {
                int nextWayPointIndex = self.NextWaypointIndex;
                nextWayPointIndex = (nextWayPointIndex + 1) % world.Waypoints.Length;
                int nwx = world.Waypoints[nextWayPointIndex][0];
                int nwy = world.Waypoints[nextWayPointIndex][1];
                MyWay tempWay = FindOptimalWay(self.X, self.Y, nwx, nwy);
                if (tempWay != null) way = tempWay;
                
            }
        }


        private bool IsNearWallsEdges(double forwardWallDetect)//если в данный момент несемся на стену
        {
            //return false;
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
            { return true; }
            return false;

        }



        private bool IsForwardWallsEdges()//если в данный момент несемся на стену
        {   if (speedModule == 0.0D) return false;
            double c = game.TrackTileSize;
            Vector2[] corners = new Vector2[] { new Vector2(0,0),new Vector2(0,c), new Vector2(c,0), new Vector2(c, c) };
            Vector2 myCoord = new Vector2(TransormToCellCoord(self.X), TransormToCellCoord(self.Y));
            Vector2 corner = new Vector2(Math.Sign(self.SpeedX)>0? c :0, Math.Sign(self.SpeedY) > 0 ? c : 0);
            double dx = myCoord.x - corner.x ;
            double dy = myCoord.y - corner.y ;
            double x,y;
            if (Math.Abs(dx) > Math.Abs(dy))
            { if (dx == 0.0D) return false;
                x = corner.x;
                y = myCoord.y + (corner.x) * (dy) / (dx) ;
            }
            else
            {
                if (dy == 0.0D) return false;
                y = corner.y;
                x = myCoord.x + (corner.y) * (dx) / (dy);
            }

            double distance = Hypot(x - corner.x, y - corner.y);
            double ticks = distance / speedModule;

            double ang = self.AngularSpeed * ticks;
            x += speedModule * Math.Cos(ang);
            y += speedModule * Math.Sin(ang);

            foreach (Vector2 testCorner in corners) {
                if (game.TrackTileMargin + game.CarWidth / 2 > Hypot(x - testCorner.x, y - testCorner.y)) return true;
            }
            //return Math.Abs(Math.Cos(self.Angle) * speedModule - self.SpeedX) +Math.Abs(Math.Sin(self.Angle) * speedModule - self.SpeedY) < 0.1D;
            return false;

        }


        private bool OnMyLine(double px, double py, double x1, double y1, double ex, double ey, double p)
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


  

        private void FireinEnemy(Move move)
        {
            if (WhoOnMyFireLine(IsEnemyOnMyFireLine) != null) move.IsThrowProjectile = true;
           
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

        MyWay FindOptimalWay(double px, double py, int wx, int wy)
        {
            //подготовка волновой карты
            CopyMapToWaveMap();
            int x = Transform(px);
            int y = Transform(py);
            myMap[x][y].waveLen = 1;// startpoint
            int lenCount = FillShortWay(wx, wy);
            //поиск кратчайшего и наименее угловатого пути
            if (lenCount <= 2) return (new MyWay(wx, wy)).CalcTargetCenter(game.TrackTileSize);//найден путь и это соседняя клетка, дальше обсчитывать бессмысленно
            MyWay[] myWay = new MyWay[lenCount];
            myWay[lenCount - 1] = (new MyWay(wx, wy, myMap[wx][wy])).CalcTargetCenter(game.TrackTileSize);
            bool isNearWall = IsNearWallsEdges(FORWARDWALLDETECT);//внутри нашей клетки мы видим стену перед собой
            //собираем путь по центрам клеток
            for (int i = lenCount - 1; i > 1; i--)
                myWay[i - 1] = FindAround(i, myWay).CalcTargetCenter(game.TrackTileSize);

            bool thruCurrentWay = false;//проверяем что наш путь проходит через ожидаемый вейпоинт (на случай второй итерации при которой мы целимся на будущий вейпоинт)

            for (int i = lenCount - 1; i >= 1; i--)
                thruCurrentWay |= (myWay[i].x == self.NextWaypointX && myWay[i].y == self.NextWaypointY);
            if (!thruCurrentWay) return null;//отсекаем этот путь 

            if (lenCount > 3)   angleAfter3Cells = self.GetAngleTo(myWay[3].target.x, myWay[3].target.y);//острота угла поворота

            for (int i = lenCount > 8 ? 8 : lenCount - 1; i > 0; i--)
                CorrectInOutWayPoint(myWay[i].CorrectCenterPoint(game.TrackTileSize));//корректируем центры масс клеток

            // bool isNearWall = IsNearWallsEdges(5);

            if (!isNearWall)
                for (int i = lenCount - 1; i > 1; i--)
                    if (isNoWallAtLine(px, py, myWay[i], i)) return myWay[i];//оптимизируем путь удаляя ненужные клетки  
            return myWay[1];
        }



        bool isNoWallAtLine(double x0, double y0, MyWay myWay, int lenCount)//рисует линию проверяя на каждом шаге на тайл и касание внутренних углов/стен тайла
        {
            //            double x1 = myWay.GetCenterX(game.TrackTileSize);
            //            double y1 = myWay.GetCenterY(game.TrackTileSize);
            double x1 = myWay.target.x;
            double y1 = myWay.target.y;
            double x = x0, y = y0;
            double dx = x1 - x0;
            double dy = y1 - y0;
            int xw, yw;
            double lineStep = game.TrackTileMargin;
            double distance, length = self.GetDistanceTo(myWay.target.x, myWay.target.y) / lineStep;
            if (length > 10 * game.TrackTileSize / lineStep )//4 cubes 10 circles
                length = 10 * game.TrackTileSize / lineStep;
            //int i = 0, l;
            distance = 0.0D;
            while (length > 1.0D)
            {
                if (Math.Abs(dx) > Math.Abs(dy))
                {
                    //l = (int)Math.Abs(dx/LINESTEP);
                    x += dx > 0 ? lineStep : -lineStep;
                    y = y0 + (x - x0) * (dy) / (dx);
                }
                else
                {
                    //l = (int)Math.Abs(dy / LINESTEP);
                    y += dy > 0 ? lineStep : -lineStep;
                    x = x0 + (y - y0) * (dx) / (dy);
                }
                length -= 1.0D;
                xw = Transform(x);
                yw = Transform(y);
                WaveMapCell myTile = new WaveMapCell();
                int w = WaveAt(xw, yw, myTile);//если линия попала внутрь стены
                if (w == 0 || w > lenCount) return false;
                distance += 1.5D;
                if (IsInnerWallIn(x - DTransoform0(xw), y - DTransoform0(yw), distance, myTile)) return false;//если линия касается внутренней стены тайла

            }
            return true;

        }

        private bool IsInnerWallIn(double v1, double v2,  double distance, WaveMapCell myTile)
        {
            return CoordsInEdgeRadius(v1, v2, game.CarWidth / 2 + game.TrackTileMargin - distance, myTile);
        }

        private bool CoordsInEdgeRadius(double v1, double v2, double radius, WaveMapCell myTile)
        {
            if (Hypot(v1, v2) < radius) return true;
            if (Hypot(game.TrackTileSize - v1, game.TrackTileSize - v2) < radius) return true;
            if (Hypot(v1, game.TrackTileSize - v2) < radius) return true;
            if (Hypot(game.TrackTileSize - v1, v2) < radius) return true;
            if (!wallTile.ContainsKey(myTile.tile)) return true;
            Direction[] dirs = wallTile[myTile.tile];

            foreach (Direction dir in dirs) {
                if (dir == Direction.Up) if (v2 < radius) return true;
                if (dir == Direction.Down) if (v2 > radius) return true;
                if (dir == Direction.Right) if (v1 < radius) return true;
                if (dir == Direction.Left) if (v1 > radius) return true;
            }

            return false;
        }


        MyWay FindAround(int i, MyWay[] tempWay)//i-- 
        {
            int x = tempWay[i].x;
            int y = tempWay[i].y;
            Direction[] dir = new Direction[] { };

            WaveMapCell thisTile = tempWay[i].waveTile;
            if (fromTile.ContainsKey(thisTile.tile))
                dir = fromTile[thisTile.tile];

            MyWay myWay = DirectionsContainsWave(i, x, y, dir);


            tempWay[i].dirIn = myWay.dirOut;
            tempWay[i].isInDirected = true;
            WaveMapCell myTile = new WaveMapCell();
            WaveAt(myWay.x, myWay.y, myTile);
            myWay.waveTile = myTile;
            return myWay;
        }

        private MyWay DirectionsContainsWave(int i, int x, int y, Direction[] dir)
        {
            double ts = game.TrackTileSize;
            MyWay myWay = new MyWay(x, y);
            if (dir.Contains(Direction.Right) && isNextWaveAt(x + 1, y, i)) { MyWay otherWay = new MyWay(x + 1, y); otherWay.dirOut = Direction.Left; myWay.otherWays.Add(otherWay.CalcTargetCenter(ts)); }
            if (dir.Contains(Direction.Left) && isNextWaveAt(x - 1, y, i)) { MyWay otherWay = new MyWay(x - 1, y); otherWay.dirOut = Direction.Right; myWay.otherWays.Add(otherWay.CalcTargetCenter(ts)); }
            if (dir.Contains(Direction.Down) && isNextWaveAt(x, y + 1, i)) { MyWay otherWay = new MyWay(x, y + 1); otherWay.dirOut = Direction.Up; myWay.otherWays.Add(otherWay.CalcTargetCenter(ts)); }
            if (dir.Contains(Direction.Up) && isNextWaveAt(x, y - 1, i)) { MyWay otherWay = new MyWay(x, y - 1); otherWay.dirOut = Direction.Down; myWay.otherWays.Add(otherWay.CalcTargetCenter(ts)); }
            double minAngle = 100;
            MyWay minWay = myWay;
            //var bestWay = from MyWay way in myWay.otherWays where 
            foreach (MyWay way in myWay.otherWays)
            {
                double angle = Math.Abs(self.GetAngleTo(way.target.x, way.target.y));
                if (angle < minAngle) { minWay = way; minAngle = angle; minWay.isOutDirected = true; }
            }

            return minWay;
        }


        private bool isNextWaveAt(int x, int y, int i)
        {
            return WaveAt(x, y) == i;
        }

        int WaveAt(int x, int y)
        {
            WaveMapCell myTile = new WaveMapCell();// = myMap[x][y];
            return WaveAt(x, y, myTile);
        }


        int WaveAt(int x, int y, WaveMapCell myTile)
        {
            if (x < 0 || y < 0 || x >= myMap.Length || y >= myMap[x].Length) return 0;
            myTile.tile = myMap[x][y].tile;
            myTile.waveLen = myMap[x][y].waveLen;
            if (!fromTile.ContainsKey(myTile.tile)) return 0;
            return myTile.waveLen;
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




        int FillOneWave(int wx, int wy)
        {
            for (int x = 0; x < myMap.Length; x++)
                for (int y = 0; y < myMap[x].Length; y++)
                {
                    WaveMapCell myTile = myMap[x][y];
                    if (myTile.waveLen == 0) continue;
                    if (myTile.isFillAround) continue;
                    if (myTile.isWall()) continue;
                    if (x == wx && y == wy) return myTile.waveLen;//кратчайший путь до точки найден
                    FillAround(x, y, myTile);
                    myMap[x][y] = myTile;
                }
            return 0;
        }

        private void FillAround(int x, int y, WaveMapCell myTile)
        {
            if (!fromTile.ContainsKey(myTile.tile)) return;
            Direction[] openDirs = fromTile[myTile.tile];
            foreach (Direction dir in openDirs)
            {
                FillOpenDirection(x, y, dir, myTile.waveLen + 1);
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
            WaveMapCell myTile = myMap[x][y];
            if (!fromTile.ContainsKey(myTile.tile)) return false;
            if (myTile.waveLen > 0) return false;
            myTile.waveLen = lenCount;
            myMap[x][y] = myTile;
            return true;

        }

        void CopyMapToWaveMap()
        {
            myMap = new WaveMapCell[world.TilesXY.Length][];
            for (int x = 0; x < myMap.Length; x++)
            {
                myMap[x] = new WaveMapCell[world.TilesXY[x].Length];
                for (int y = 0; y < myMap[x].Length; y++)
                {
                    WaveMapCell myTile = new WaveMapCell();
                    myTile.tile = world.TilesXY[x][y];
                    myTile.isFillAround = false;
                    myTile.waveLen = 0;
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
            if (stopTickCount > MAXSTOPCOUNT)
            {
                currentState = MovingState.BACKWARD;
                stopTickCount = 0;
                stateTickCount = 0;
                if (IsWallCollisionDetect())
                {
                    //errorBlockCount = MAXERRORBLOCKS;
                }
            }
            if (currentState == MovingState.BACKWARD)
            {
                stateTickCount++;
                if (stateTickCount > MAXBACKTICKS)
                {
                    currentState = MovingState.FORWARD;
                }
            }

        }

        private bool IsWallCollisionDetect()
        {
            double f = Hypot(game.CarHeight, game.CarWidth) + game.TrackTileMargin;
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

        private Car WhoOnMyFireLine(Func<Car, bool> condition)            
        {
            var search = from Car enemy in world.Cars
                         where
                         condition(enemy)
                         select enemy;
            return search.FirstOrDefault();

        }

        private bool IsEnemyOnMyFireLine(Car enemy)
        {
            double distance = self.GetDistanceTo(enemy);
            if (distance > game.TrackTileSize * 7) return false;
            //узнаем количество тиков полета шайб до врага
            double ticksPerWasher = (distance / game.WasherInitialSpeed);
            //вычисляем траекторию врага через это количество тиков
            Vector2 fEnemy = new Vector2(enemy.X + enemy.SpeedX * ticksPerWasher, enemy.X + enemy.SpeedX * ticksPerWasher);

            /*   getSideWasherAngle
            – Returns - Возвращает модуль отклонения направления полёта двух шайб от направления
            кодемобиля.Направление третьей шайбы совпадает с направлением кодемобиля.*/
            return enemy.Durability > 0.0D && !enemy.IsFinishedTrack && !enemy.IsTeammate && Math.Abs(self.GetAngleTo(fEnemy.x, fEnemy.y)) < game.SideWasherAngle/3 && Math.Abs(self.GetAngleTo(enemy)) < 0.3;
                                     
        }

        private bool IsEnemyBehindMe(Car enemy)
        {
            double enemySpeedModule = Hypot(enemy.SpeedX, enemy.SpeedY);
            if (enemySpeedModule <= MINSPEED) return false;//враг стоит на месте
            double distance = self.GetDistanceTo(enemy);
            if (distance > game.TrackTileSize * 7) return false;//враг далеко
            double ticksPerWasher = distance / enemySpeedModule;
            Vector2 fEnemy = new Vector2(enemy.X + enemy.SpeedX * ticksPerWasher, enemy.X + enemy.SpeedX * ticksPerWasher);
            double fRadius = Hypot(fEnemy.x - self.X, fEnemy.y - self.Y);
            return !enemy.IsTeammate && fRadius < game.OilSlickRadius;
        }


        double Hypot(double x, double y) { return Math.Sqrt(x * x + y * y); }
        int Transform(double x) { return (int)((x) / game.TrackTileSize); }
        double InvTransoform(int x) { return (double)(x + 0.5D) * game.TrackTileSize; }
        double DTransoform0(int x) { return (double)(x) * game.TrackTileSize; }
        double TransormToCellCoord(double x) {int myX = Transform(x); return x - DTransoform0(myX);   }


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

        private void FillWallTileTable()
        {
            wallTile.Add(TileType.Vertical, new Direction[] { Direction.Right, Direction.Left });
            wallTile.Add(TileType.Horizontal, new Direction[] { Direction.Up, Direction.Down });
            wallTile.Add(TileType.Crossroads, new Direction[] { });
            wallTile.Add(TileType.BottomHeadedT, new Direction[] { Direction.Up });
            wallTile.Add(TileType.LeftBottomCorner, new Direction[] { Direction.Left, Direction.Down});
            wallTile.Add(TileType.LeftHeadedT, new Direction[] { Direction.Right});
            wallTile.Add(TileType.LeftTopCorner, new Direction[] { Direction.Left, Direction.Up });
            wallTile.Add(TileType.RightBottomCorner, new Direction[] { Direction.Right, Direction.Down });
            wallTile.Add(TileType.RightHeadedT, new Direction[] { Direction.Left });
            wallTile.Add(TileType.RightTopCorner, new Direction[] { Direction.Right, Direction.Up });
            wallTile.Add(TileType.TopHeadedT, new Direction[] { Direction.Down });

        }


        class MyWay
        {
            public int x, y;
            public WaveMapCell waveTile;
            public Direction dirOut;
            public Direction dirIn; //направление входа в клетку
            public bool isInDirected = false;//отвечает за то что dirOut посчитан и заполнен (dirOut перечисление и не может быть null)
            public bool isOutDirected = false;
            public Vector2 target;
            public List<MyWay> otherWays = new List<MyWay>();

            public MyWay()
            {

            }

            public MyWay(int x, int y)
            {
                this.x = x; this.y = y;
            }

            public MyWay(int x, int y, WaveMapCell myTile)
            {
                this.x = x; this.y = y; this.waveTile = myTile;
            }

            public double GetCenterX(double TrackTileSize)
            {
                return (double)(x + 0.5D) * TrackTileSize;
            }

            public double GetCenterY(double TrackTileSize)
            {
                return (double)(y + 0.5D) * TrackTileSize;
            }
            public MyWay CalcTargetCenter(double TrackTileSize) {
                this.target = new Vector2(this.GetCenterX(TrackTileSize),this.GetCenterY(TrackTileSize));
                CorrectCenterPoint(TrackTileSize);
                return this;
            }

            public MyWay CorrectCenterPoint(double TrackTileSize)
            {
                double cornerTileOffset = CORNERCORRECTION * TrackTileSize;
                if (target == null) return this;
                if (this.waveTile == null) return this;
                switch (this.waveTile.tile)
                {
                    case TileType.LeftTopCorner:
                        target.x += cornerTileOffset;
                        target.y += cornerTileOffset;
                        break;
                    case TileType.RightTopCorner:
                        target.x -= cornerTileOffset;
                        target.y += cornerTileOffset;
                        break;
                    case TileType.LeftBottomCorner:
                        target.x += cornerTileOffset;
                        target.y -= cornerTileOffset;
                        break;
                    case TileType.RightBottomCorner:
                        target.x -= cornerTileOffset;
                        target.y -= cornerTileOffset;
                        break;
                    default:
                        break;
                }
                return this;
            }


        }

        public class Vector2
        {
            public double x, y;

            public Vector2()
            {
            }

            public Vector2(double x, double y) { this.x = x; this.y = y; }
        }

        public class WaveMapCell// клетка в формате волнового алгоритма
        {
            public TileType tile;
            public int waveLen;//длина волны
            public bool isFillAround; //волна уже обсчитана/обработана вокруг
            public bool isWall()
            {
                return tile == TileType.Empty;
            }
        }
    }




}
