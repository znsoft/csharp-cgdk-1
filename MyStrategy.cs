using System;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk {
    public sealed class MyStrategy : IStrategy {
        public void Move(Car self, World world, Game game, Move move) {
            move.EnginePower = 1.0D;
            move.IsThrowProjectile = true;
            move.IsSpillOil = true;

            if (world.Tick > game.InitialFreezeDurationTicks) {
                move.IsUseNitro = true;
            }

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
            double speedModule = Hypot(self.SpeedX, self.SpeedY);
            
            move.WheelTurn = angleToWaypoint * 32.0D / Math.PI;
            move.EnginePower = (0.75D);

            if (speedModule * speedModule * Math.Abs(angleToWaypoint) > 2.5D * 2.5D * Math.PI)
            {
                move.IsBrake = true;
            }

        }

        double Hypot(double x, double y) { return Math.Sqrt(x * x + y * y); }

    }


}
