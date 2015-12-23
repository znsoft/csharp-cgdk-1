using System;
using System.Collections;
using System.Linq;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.Model;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Globalization;

namespace Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk
{
    public sealed class MyStrategy : IStrategy
    {
        const int MAXSTOPCOUNT = 12;
        const double MINSPEED = 0.09D;
        const int MAXBACKTICKS = 300;
        const double PRECALCDIRECTIONOFFSET = 23;
        const int MAXWAYITERATIONS = 220;//максимально возможная длина пути (ограничить проц время)
        const int MAXERRORBLOCKS = 8;
        const double PRE_TURN_SPEEDMUL = 15.5D;
        const double CORNERCORRECTION = -0.25D;
        const double FORWARDWALLDETECT = 15.0D;
        const double LINESTEP = 15.0D;
        const double BREAKTRESHHOLD = 4000.0D;

        enum MovingState
        {
            FORWARD,
            BACKWARD,
            STOP
        }

        MovingState currentState = MovingState.FORWARD;
        int stateTickCount = 0;

        double carAccel,      frictionMove,            frictionLenght,        frictionCross,
                    angleSpeedFactor,            frictionAngle,            frictionMaxAngleSpeed;

        Dictionary<TileType, Direction[]> fromTile = new Dictionary<TileType, Direction[]>();
        Dictionary<TileType, Direction[]> wallTile = new Dictionary<TileType, Direction[]>();
        TileType[] ForwardDirectionTiles = new TileType[] { TileType.Horizontal, TileType.Vertical };

        Car self;
        World world;
        Game game;
        Move move;
        double speedModule;
        private int stopTickCount = 0;
        MyWay way;
        WaveMapCell[][] myMap;
        //int errorBlockCount = 0; //если столкнулись со стеной то несколько блоков едем "осторожно" без предсказаний
        private TcpClient client = null;
        //private readonly BinaryReader reader;
        StreamReader reader;
        StreamWriter writer;


        //private readonly BinaryWriter writer;
        private const int BufferSizeBytes = 1 << 20;


        public MyStrategy()
        {
            FillFromTileTable();
            FillWallTileTable();
        }


        public void Move(Car self, World world, Game game, Move move)
        {
            connectToVisualizer();
            move.EnginePower = 1.0D;// (0.95D);
            Construct(self, world, game, move);
            CalcInitOnce(1);
            AnalyzeCurrentSpeedAndState();
            double distance = self.GetDistanceTo(InvTransoform(self.NextWaypointX), InvTransoform(self.NextWaypointY));
            way = GetFwdWay();
            VisualizeSendLine(self.X, self.Y, way.target.x, way.target.y, 5);

            double angleToWaypoint = self.GetAngleTo(way.target.x, way.target.y);
            move.WheelTurn = angleToWaypoint * 32.0D / Math.PI;

            if (speedModule * speedModule * speedModule * Math.Abs(angleToWaypoint) > BREAKTRESHHOLD)
            {
                //  move.IsBrake = true;
            }
            if (self.GetDistanceTo(way.target.x, way.target.y) > game.TrackTileSize * 4 && angleToWaypoint < 2) move.IsUseNitro = true;

            BackMove(move);
            FireinEnemy(move);
            OilFireEnemy(move);

        }


        void connectToVisualizer()
        {
            try
            {
                if (client != null) return;
                client = new TcpClient("127.0.0.1", 8901)
                {
                    SendBufferSize = BufferSizeBytes,
                    ReceiveBufferSize = BufferSizeBytes,
                    NoDelay = true
                };

                reader = new StreamReader(client.GetStream());
                writer = new StreamWriter(client.GetStream());
            }
            catch (Exception e) { client = null; }

        }

        void VisualizerWrite(String command)
        {
            if (writer == null || client == null) return;
            try
            {
                writer.WriteLine(command);
                writer.Flush();
            }
            catch (Exception e) { client = null; }
        }

        void VisualizeSendLine(double x, double y, double x1, double y1, double c)
        {
            NumberFormatInfo nfi = CultureInfo.CreateSpecificCulture("en-US").NumberFormat;
            VisualizerWrite(string.Format("{0:F0},{1:F0},{2:F0},{3:F0},{4}", x, y, x1, y1, c.ToString("N3", nfi)));

        }

        private void BackMove(Move move)
        {
            if (currentState == MovingState.BACKWARD)
            {
                if (stateTickCount < MAXBACKTICKS / 3)
                    move.EnginePower = -1;
                //move.IsUseNitro = true;
                move.IsBrake = false;
                if (stateTickCount < MAXBACKTICKS / 2) move.WheelTurn = -move.WheelTurn * 1.8D;
            }
        }

        private void OilFireEnemy(Move move)
        {
            TileType[] notNeedOil = new TileType[] { TileType.Crossroads, TileType.Horizontal, TileType.Vertical };
            TileType currentTile = world.TilesXY[Transform(self.X)][Transform(self.Y)];
            if (!notNeedOil.Contains(currentTile) || WhoOnMyFireLine(IsEnemyBehindMe) != null)
            {
                move.IsSpillOil = true;
            }
        }

        private void CorrectInOutWayPoint(MyWay way, ref double nextWaypointX, ref double nextWaypointY, double speed)
        {

            if (!way.isInDirected && !way.isOutDirected) return;
            double size = speed * PRECALCDIRECTIONOFFSET + game.TrackTileSize / 2;

            Vector2 next = new Vector2(nextWaypointX, nextWaypointY);
            Vector2 offset = null;
            if (way.isInDirected) offset = GetInOffset(way.dirIn, next, size);
            if (offset == null && way.isOutDirected)
                offset = GetOutOffset(way.dirOut, next, size);
            if (offset == null) return;
            nextWaypointX += offset.x;
            nextWaypointY += offset.y;
        }

        private void CorrectInOutWayPoint(MyWay way)
        {

            if (!way.isInDirected && !way.isOutDirected) return;
            if (way.target == null) way.CalcTargetCenter(game.TrackTileSize);
            double size = speedModule * PRECALCDIRECTIONOFFSET + game.TrackTileSize / 2;

            Vector2 next = way.target;
            Vector2 offset = null;
            if (way.isInDirected) offset = GetInOffset(way.dirIn, next, size);
            if (offset == null && way.isOutDirected)
                offset = GetOutOffset(way.dirOut, next, size);
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
                MyWay tempWay = FindWay(self.X, self.Y, nwx, nwy);
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


        MyWay GetFwdWay()
        {
            MyWay[] tWay = CalcPathToWayPoints();
            int i = GenerateWayinPath(tWay);
            if (i > 0 && i < 4 && speedModule < 10) move.IsBrake = true;
            if (i > 0) return tWay[i];
            CorrectInOutWayPoint(tWay[1].CorrectCenterPoint(game.TrackTileSize));
            return tWay[1];
        }


        int GenerateWayinPath(MyWay[] tWay)
        {
            int lenCount = tWay.Length;
            //int l = lenCount - 1;
            Vector2 xy = new Vector2(self.X, self.Y);
            int len, maxLenForward = 16;
            for (int i = lenCount > 8 ? 8 : lenCount - 1; i > 1; i--)
            {
                len = SimulateFwdTicks(tWay[i], maxLenForward);
                if (len > 0) return i;
            }
            return 0;
        }


        MyWay[] CalcPathToWayPoints()
        {

            int nextWayPointIndex = self.NextWaypointIndex;
            nextWayPointIndex = (nextWayPointIndex + 1) % world.Waypoints.Length;
            int nwx = world.Waypoints[nextWayPointIndex][0];
            int nwy = world.Waypoints[nextWayPointIndex][1];
            MyWay[] tempWay = FindPath(self.X, self.Y, nwx, nwy);
            if (tempWay == null)
                tempWay = FindPath(self.X, self.Y, self.NextWaypointX, self.NextWaypointY);
            return tempWay;
        }



        ///значения высчитываются один раз

        void CalcInitOnce(double dt)
        {
            carAccel = carAcceleration(self, game, dt);
            frictionMove = Math.Pow(1 - game.CarMovementAirFrictionFactor, dt);
            frictionLenght = game.CarLengthwiseMovementFrictionFactor * dt;
            frictionCross = game.CarCrosswiseMovementFrictionFactor * dt;

            angleSpeedFactor = game.CarAngularSpeedFactor;
            frictionAngle = Math.Pow(1 - game.CarRotationFrictionFactor, dt);
            frictionMaxAngleSpeed = game.CarRotationFrictionFactor * dt;
        }

        private static double carAcceleration(Car car, Game game, double dt)
        {
            switch (car.Type)
            {
                case CarType.Buggy:
                    return game.BuggyEngineForwardPower / game.BuggyMass * dt;
                case CarType.Jeep:
                    return game.JeepEngineForwardPower / game.JeepMass * dt;
            }
            return 0;
        }

        double signLimitChange(double idealWheelTurn, double wheelTurn, double CarWheelTurnChangePerTick) {
            if (wheelTurn < idealWheelTurn)
                wheelTurn += CarWheelTurnChangePerTick;
            if (wheelTurn > idealWheelTurn)
                wheelTurn -= CarWheelTurnChangePerTick;
            return wheelTurn;
        }
/*
        public void Iteration(int ticks)
        {
            Vector2 pos = new Vector2(self.X, self.Y);
            Vector2 spd = new Vector2(self.SpeedX, self.SpeedY);
            Vector2 dir;

            for (int i = 0; i < ticks / dt; i++)
            {
                wheelTurn = signLimitChange(idealWheelTurn, wheelTurn, game.CarWheelTurnChangePerTick);
                enginePower = signLimitChange(idealEnginePower, enginePower, game.CarEnginePowerChangePerTick);

                if (nitroTicks > 0)
                {
                    enginePower = 2;
                    nitroTicks--;
                }

                double baseAngleSpeed = wheelTurn * game.CarAngularSpeedFactor * spd.Dot(dir);

                angle += angleSpeed * dt;
                angle = angle.AngleNormalize();
                angleSpeed = baseAngleSpeed + (angleSpeed - baseAngleSpeed) * frictionAngle;
                angleSpeed -= limit(angleSpeed - baseAngleSpeed, frictionMaxAngleSpeed);

                dir = Vector2.sincos(angle);

                Vector2 accel = dir * (enginePower * brake * carAccel);

                pos = pos + spd * dt;
                spd = (spd + accel) * frictionMove;
                spd = spd - dir * limit(spd.Dot(dir), frictionLenght) - dir.PerpendicularLeft() * limit(spd.Cross(dir), frictionCross);
            }
        }

        */


        MyWay[] FindPath(double px, double py, int wx, int wy)
        {
            CopyMapToWaveMap();
            int x = Transform(px);
            int y = Transform(py);
            myMap[x][y].waveLen = 1;

            int lenCount = FillShortWay(wx, wy);
            MyWay[] myWay = new MyWay[lenCount];
            myWay[0] = (new MyWay(x, y)).CalcTargetCenter(game.TrackTileSize);
            myWay[0].target.x = px;
            myWay[0].target.y = py;
            myWay[lenCount - 1] = (new MyWay(wx, wy, myMap[wx][wy])).CalcTargetCenter(game.TrackTileSize);
            for (int i = lenCount - 1; i > 1; i--)
                myWay[i - 1] = FindAround(i, myWay).CalcTargetCenter(game.TrackTileSize);

            bool thruCurrentWay = false;

            for (int i = lenCount - 1; i >= 1; i--)
                thruCurrentWay |= (myWay[i].x == self.NextWaypointX && myWay[i].y == self.NextWaypointY);
            if (!thruCurrentWay) return null;

            return myWay;
        }


        int SimulateFwdTicks(MyWay myWay, int maxLenForward)
        {
            double k = 2;
            double speedx = self.SpeedX;
            double speedy = self.SpeedY;
            double speed = Hypot(speedx, speedy);
            double x = self.X;
            double y = self.Y;
            double tx = myWay.target.x;
            double ty = myWay.target.y;
            // double angspeed = self.AngularSpeed;
            double angle = self.Angle;
            double wheelTurn = move.WheelTurn;
            double enginePower = move.EnginePower;
            bool thruWay = false;
            for (int i = 0; i < maxLenForward; i++)
            {
                VisualizeSendLine(x, y, x + speedx, y + speedy, 1);
                y += speedy; x += speedx;
                thruWay |= (Transform(x) == myWay.x && Transform(y) == myWay.y);
                if (thruWay)
                {
                    myWay.target.x = x;
                    myWay.target.y = y;
                    return i;
                }
                if (IsWallForecastDetect(new Vector2(x, y)))
                    return 0;

                speed = Hypot(speedx, speedy);
                double speedAngle = speed < 2 ? angle : Math.Atan2(speedy, speedx);

                //if (enginePower < 0.01 && enginePower > -0.01) speed -= speed > 0 ? game.CarEnginePowerChangePerTick : 0;
                if (speed < 33 * 2 && enginePower > 0) speed += game.CarEnginePowerChangePerTick * k;
                //if (speed > 0 && enginePower < 0) speed -= game.CarEnginePowerChangePerTick;
                wheelTurn = GetAngleFromTo(x, y, speedAngle, tx, ty);

                if (wheelTurn > 0.01)
                    speedAngle += game.CarWheelTurnChangePerTick * k * 2;

                if (wheelTurn < -0.01)
                    speedAngle -= game.CarWheelTurnChangePerTick * k * 2;

                speedx = Math.Cos(speedAngle) * speed * k;
                speedy = Math.Sin(speedAngle) * speed * k;



            }
            return 0;

        }




        private bool IsWallForecastDetect(Vector2 testCoord)
        {
            double radius = Hypot(self.Width, self.Height) / 2 + game.TrackTileMargin;
            double x = TransormToCellCoord(testCoord.x);
            double y = TransormToCellCoord(testCoord.y);
            if (Hypot(x, y) < radius) return true;
            if (Hypot(game.TrackTileSize - x, game.TrackTileSize - y) < radius) return true;
            if (Hypot(x, game.TrackTileSize - y) < radius) return true;
            if (Hypot(game.TrackTileSize - x, y) < radius) return true;
            int tx = Transform(testCoord.x) % (world.TilesXY.Length - 1);
            int ty = Transform(testCoord.y) % (world.TilesXY[tx].Length - 1);
            TileType myTile = world.TilesXY[tx][ty];
            if (!wallTile.ContainsKey(myTile)) return true;
            Direction[] dirs = wallTile[myTile];

            foreach (Direction dir in dirs)
            {
                if (dir == Direction.Up) if (y < radius) return true;
                if (dir == Direction.Down) if (y > game.TrackTileSize - radius) return true;
                if (dir == Direction.Right) if (x > game.TrackTileSize - radius) return true;
                if (dir == Direction.Left) if (x < radius) return true;
            }

            return false;

        }

        int GetMoveFwdTicks(MyWay myWay, int maxLenForward, Move move)
        {
            double k = 2;
            double speedx = self.SpeedX;
            double speedy = self.SpeedY;
            double speed = Hypot(speedx, speedy);
            double x = self.X;
            double y = self.Y;
            double tx = myWay.target.x;
            double ty = myWay.target.y;
            // double angspeed = self.AngularSpeed;
            double angle = self.Angle;
            double wheelTurn = move.WheelTurn;
            double enginePower = move.EnginePower;
            bool thruWay = false;
            for (int i = 0; i < maxLenForward; i++)
            {
                VisualizeSendLine(x, y, x + speedx, y + speedy, 1);
                y += speedy; x += speedx;
                thruWay |= (Transform(x) == myWay.x && Transform(y) == myWay.y);
                if (thruWay)
                {
                    myWay.target.x = x;
                    myWay.target.y = y;
                    return i;
                }
                if (IsWallForecastDetect(new Vector2(x, y)))
                {

                    if (thruWay)
                    {
                        myWay.target.x = x;
                        myWay.target.y = y;
                        return i;
                    }
                    return 0;
                }

                speed = Hypot(speedx, speedy);
                double speedAngle = speed < 2 ? angle : Math.Atan2(speedy, speedx);

                //if (enginePower < 0.01 && enginePower > -0.01) speed -= speed > 0 ? game.CarEnginePowerChangePerTick : 0;
                if (speed < 33 * 2 && enginePower > 0) speed += game.CarEnginePowerChangePerTick * k;
                //if (speed > 0 && enginePower < 0) speed -= game.CarEnginePowerChangePerTick;
                wheelTurn = GetAngleFromTo(x, y, speedAngle, tx, ty);

                if (wheelTurn > 0.01)
                    speedAngle += game.CarWheelTurnChangePerTick * k * 2;

                if (wheelTurn < -0.01)
                    speedAngle -= game.CarWheelTurnChangePerTick * k * 2;

                speedx = Math.Cos(speedAngle) * speed * k;
                speedy = Math.Sin(speedAngle) * speed * k;



            }
            if (thruWay)
            {
                myWay.target.x = x;
                myWay.target.y = y;
                return maxLenForward;
            }
            return thruWay ? maxLenForward : 0;

        }


        private MyWay GenerateNewWay(MyWay way)
        {
            int wayCount = 4;
            Vector2[] shift = new Vector2[] {
                new Vector2(1, 1),
                new Vector2(-1, 1),
                new Vector2(-1, -1),
                new Vector2(1, -1),
                new Vector2(1, 1)

            };
            int[] lens = new int[wayCount];
            MyWay[] ways = new MyWay[wayCount];
            ways[0] = way;

            int max = 0, j;
            int ticksForward = 16;
            for (int i = 1; i < ways.Length; i++)
            {
                ways[i] = new MyWay(way.x, way.y);
                double xs = shift[i].x;
                double ys = shift[i].y;
                xs *= game.TrackTileSize / 3;
                ys *= game.TrackTileSize / 3;
                ways[i].target = new Vector2(ways[i].GetCenterX(game.TrackTileSize) + xs, ways[i].GetCenterY(game.TrackTileSize) + ys);
            }
            for (int i = 0; i < ways.Length; i++)
            {
                double d = self.GetDistanceTo(ways[i].target.x, ways[i].target.y) / 2;
                j = GetMoveFwdTicks(ways[i], d > ticksForward ? ticksForward : (int)d, move);
                if (j > max)
                {
                    max = j; way = ways[i];
                    //VisualizeSendLine(self.X, self.Y, ways[i].target.x, ways[i].target.y);
                }

            }
            if (max < 8 && speedModule > 9 && IsWallForecastDetect(way.target)) { move.IsBrake = true; Console.WriteLine(max); }
            return way;
        }


        MyWay FindWay(double px, double py, int wx, int wy)
        {
            CopyMapToWaveMap();
            int x = Transform(px);
            int y = Transform(py);
            myMap[x][y].waveLen = 1;// startpoint
            int lenCount = FillShortWay(wx, wy);
            if (lenCount <= 2) return (new MyWay(wx, wy)).CalcTargetCenter(game.TrackTileSize);//найден путь и это соседняя клетка
            MyWay[] myWay = new MyWay[lenCount];
            myWay[lenCount - 1] = (new MyWay(wx, wy, myMap[wx][wy])).CalcTargetCenter(game.TrackTileSize);

            for (int i = lenCount - 1; i > 1; i--)
            {
                myWay[i - 1] = FindAround(i, myWay).CalcTargetCenter(game.TrackTileSize);
                CorrectInOutWayPoint(myWay[i - 1]);
            }

            bool thruCurrentWay = false;

            for (int i = lenCount - 1; i >= 1; i--)
                thruCurrentWay |= (myWay[i].x == self.NextWaypointX && myWay[i].y == self.NextWaypointY);
            if (!thruCurrentWay) return null;

            CorrectInOutWayPoint(myWay[lenCount - 1]);
            bool isNearWall = IsNearWallsEdges(FORWARDWALLDETECT);
            if (!isNearWall)
                for (int i = lenCount - 1; i > 1; i--)
                {

                    if (isNoWallAtLine(px, py, myWay[i], i)) return myWay[i];
                }


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


            int i = 0, l;

            while (true)
            {
                if (Math.Abs(dx) > Math.Abs(dy))
                {
                    l = (int)Math.Abs(dx / LINESTEP);
                    y = y0 + (x - x0) * (dy) / (dx);
                    x += dx > 0 ? LINESTEP : -LINESTEP;
                }
                else
                {
                    l = (int)Math.Abs(dy / LINESTEP);
                    x = x0 + (y - y0) * (dx) / (dy);
                    y += dy > 0 ? LINESTEP : -LINESTEP;
                }
                if (++i >= l) break;
                xw = Transform(x);
                yw = Transform(y);
                WaveMapCell myTile = new WaveMapCell();
                int w = WaveAt(xw, yw, myTile);
                if (w == 0 || w > lenCount) return false;
                if (IsInnerWallIn(x - DTransoform0(xw), y - DTransoform0(yw), myTile)) return false;//внутренние стены тайла
            }
            return true;

        }

        private bool IsInnerWallIn(double v1, double v2, WaveMapCell myTile)
        {
            return CoordsInEdgeRadius(v1, v2, game.CarWidth + game.TrackTileSize / 10);
        }


        private bool CoordsInEdgeRadius(double v1, double v2, double radius)
        {
            if (Hypot(v1, v2) < radius) return true;
            if (Hypot(game.TrackTileSize - v1, game.TrackTileSize - v2) < radius) return true;
            if (Hypot(v1, game.TrackTileSize - v2) < radius) return true;
            if (Hypot(game.TrackTileSize - v1, v2) < radius) return true;
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
                                                                  // if (myTile.tile == TileType.Unknown) { wx = x; wy = y; return myTile.waveLen; }
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
            return !enemy.IsTeammate && Math.Abs(self.GetAngleTo(enemy)) < 0.1D
                                     && self.GetDistanceTo(enemy) < game.TrackTileSize * 2;
        }

        private bool IsEnemyBehindMe(Car enemy)
        {
            return !enemy.IsTeammate && Math.Abs(self.GetAngleTo(enemy)) > 2.9D
                                     && self.GetDistanceTo(enemy) < game.TrackTileSize * 2;
        }


        double Hypot(double x, double y) { return Math.Sqrt(x * x + y * y); }
        int Transform(double x)
        {
            if (x < 0) return 0;
            return (int)((x) / game.TrackTileSize);
        }
        double InvTransoform(int x) { return (double)(x + 0.5D) * game.TrackTileSize; }
        double DTransoform0(int x) { return (double)(x) * game.TrackTileSize; }
        double TransormToCellCoord(double x) { int myX = Transform(x); return x - DTransoform0(myX); }
        private void FillWallTileTable()
        {
            wallTile.Add(TileType.Vertical, new Direction[] { Direction.Right, Direction.Left });
            wallTile.Add(TileType.Horizontal, new Direction[] { Direction.Up, Direction.Down });
            wallTile.Add(TileType.Crossroads, new Direction[] { });
            wallTile.Add(TileType.Unknown, new Direction[] { });
            wallTile.Add(TileType.BottomHeadedT, new Direction[] { Direction.Up });
            wallTile.Add(TileType.LeftBottomCorner, new Direction[] { Direction.Left, Direction.Down });
            wallTile.Add(TileType.LeftHeadedT, new Direction[] { Direction.Right });
            wallTile.Add(TileType.LeftTopCorner, new Direction[] { Direction.Left, Direction.Up });
            wallTile.Add(TileType.RightBottomCorner, new Direction[] { Direction.Right, Direction.Down });
            wallTile.Add(TileType.RightHeadedT, new Direction[] { Direction.Left });
            wallTile.Add(TileType.RightTopCorner, new Direction[] { Direction.Right, Direction.Up });
            wallTile.Add(TileType.TopHeadedT, new Direction[] { Direction.Down });

        }
        private void FillFromTileTable()
        {
            fromTile.Add(TileType.Vertical, new Direction[] { Direction.Down, Direction.Up });
            fromTile.Add(TileType.Horizontal, new Direction[] { Direction.Left, Direction.Right });
            fromTile.Add(TileType.Crossroads, new Direction[] { Direction.Left, Direction.Right, Direction.Down, Direction.Up });
            fromTile.Add(TileType.Unknown, new Direction[] { Direction.Left, Direction.Right, Direction.Down, Direction.Up });
            fromTile.Add(TileType.BottomHeadedT, new Direction[] { Direction.Left, Direction.Right, Direction.Down });
            fromTile.Add(TileType.LeftBottomCorner, new Direction[] { Direction.Right, Direction.Up });
            fromTile.Add(TileType.LeftHeadedT, new Direction[] { Direction.Left, Direction.Up, Direction.Down });
            fromTile.Add(TileType.LeftTopCorner, new Direction[] { Direction.Right, Direction.Down });
            fromTile.Add(TileType.RightBottomCorner, new Direction[] { Direction.Left, Direction.Up });
            fromTile.Add(TileType.RightHeadedT, new Direction[] { Direction.Right, Direction.Down, Direction.Up });
            fromTile.Add(TileType.RightTopCorner, new Direction[] { Direction.Left, Direction.Down });
            fromTile.Add(TileType.TopHeadedT, new Direction[] { Direction.Left, Direction.Right, Direction.Up });
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
            public MyWay CalcTargetCenter(double TrackTileSize)
            {
                this.target = new Vector2(this.GetCenterX(TrackTileSize), this.GetCenterY(TrackTileSize));
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

            void rotate(double deg)
            {
                double theta = deg / 180.0 * Math.PI;
                double c = Math.Cos(theta);
                double s = Math.Sin(theta);
                double tx = x * c - y * s;
                double ty = x * s + y * c;
                x = tx;
                y = ty;
            }

            Vector2 normalize()
            {
                if (length() == 0) return this;
                this.x *= (1.0 / length());
                this.y *= (1.0 / length());
                return this;
            }

            double dist(Vector2 v)
            {
                Vector2 d = new Vector2(v.x - x, v.y - y);
                return d.length();
            }

            double length() { return Math.Sqrt(x * x + y * y); }

            void truncate(double length)
            {
                double angle = Math.Atan2(y, x);
                x = length * Math.Cos(angle);
                y = length * Math.Sin(angle);
            }

            Vector2 ortho()
            {
                return new Vector2(y, -x);
            }

            static double dot(Vector2 v1, Vector2 v2)
            {
                return v1.x * v2.x + v1.y * v2.y;
            }
            static double cross(Vector2 v1, Vector2 v2)
            {
                return (v1.x * v2.y) - (v1.y * v2.x);
            }

            public static Vector2 operator +(Vector2 v1, Vector2 v2)
            {
                return new Vector2(v1.x + v2.x, v1.y + v2.y);
            }
            public static Vector2 operator -(Vector2 v1, Vector2 v2)
            {
                return new Vector2(v1.x - v2.x, v1.y - v2.y);
            }

            public static Vector2 operator *(Vector2 v, double r)
            {
                return new Vector2(v.x * r, v.y * r);
            }

            public static double operator *(Vector2 v1, Vector2 v2)
            {
                return v1.x * v2.x + v1.y * v2.y;
            }

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
