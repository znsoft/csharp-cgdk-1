
import java.awt.*;

import model.*;

import static java.lang.StrictMath.*;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

public final class LocalTestRendererListener {

    private Graphics graphics;
    private World world;
    private Game game;
    private Player play = null;
    private int canvasWidth;
    private int canvasHeight;

    private double left;
    private double top;
    private double width;
    private double height;

    private Socket socket = null;
    private BufferedReader in;
    private PrintWriter out;
    ServerSocket listener = null;
    ArrayList<LinePoints> linePoints = new ArrayList<LinePoints>();

    public void beforeDrawScene(Graphics graphics, World world, Game game, int canvasWidth, int canvasHeight,
            double left, double top, double width, double height) {

        updateFields(graphics, world, game, canvasWidth, canvasHeight, left, top, width, height);

        graphics.setColor(Color.GREEN);
        for (Car car : world.getCars()) {
            drawCircle(car.getX(), car.getY(), hypot(car.getWidth(), car.getHeight()) / 2.0D);
            drawLine(car.getX(), car.getY(), car.getNextWaypointX() * game.getTrackTileSize() + game.getTrackTileSize() / 2, car.getNextWaypointY() * game.getTrackTileSize() + game.getTrackTileSize() / 2);
        }
        if (play == null) {
            play = new Player();
            play.start();
        }
        graphics.setColor(Color.BLACK);
    }

    public void afterDrawScene(Graphics graphics, World world, Game game, int canvasWidth, int canvasHeight,
            double left, double top, double width, double height) {
        updateFields(graphics, world, game, canvasWidth, canvasHeight, left, top, width, height);

        graphics.setColor(Color.BLACK);
        for (LinePoints l : linePoints ){
        drawLine(l.start.getX(), l.start.getY(), l.end.getX(), l.end.getY());
        }
        linePoints.clear();
        
    }

    private void updateFields(Graphics graphics, World world, Game game, int canvasWidth, int canvasHeight,
            double left, double top, double width, double height) {
        this.graphics = graphics;
        this.world = world;
        this.game = game;

        this.canvasWidth = canvasWidth;
        this.canvasHeight = canvasHeight;

        this.left = left;
        this.top = top;
        this.width = width;
        this.height = height;
    }

    private void drawLine(double x1, double y1, double x2, double y2) {
        Point2I lineBegin = toCanvasPosition(x1, y1);
        Point2I lineEnd = toCanvasPosition(x2, y2);

        graphics.drawLine(lineBegin.getX(), lineBegin.getY(), lineEnd.getX(), lineEnd.getY());
    }

    private void fillCircle(double centerX, double centerY, double radius) {
        Point2I topLeft = toCanvasPosition(centerX - radius, centerY - radius);
        Point2I size = toCanvasOffset(2.0D * radius, 2.0D * radius);

        graphics.fillOval(topLeft.getX(), topLeft.getY(), size.getX(), size.getY());
    }

    private void drawCircle(double centerX, double centerY, double radius) {
        Point2I topLeft = toCanvasPosition(centerX - radius, centerY - radius);
        Point2I size = toCanvasOffset(2.0D * radius, 2.0D * radius);

        graphics.drawOval(topLeft.getX(), topLeft.getY(), size.getX(), size.getY());
    }

    private void fillArc(double centerX, double centerY, double radius, int startAngle, int arcAngle) {
        Point2I topLeft = toCanvasPosition(centerX - radius, centerY - radius);
        Point2I size = toCanvasOffset(2.0D * radius, 2.0D * radius);

        graphics.fillArc(topLeft.getX(), topLeft.getY(), size.getX(), size.getY(), startAngle, arcAngle);
    }

    private void drawArc(double centerX, double centerY, double radius, int startAngle, int arcAngle) {
        Point2I topLeft = toCanvasPosition(centerX - radius, centerY - radius);
        Point2I size = toCanvasOffset(2.0D * radius, 2.0D * radius);

        graphics.drawArc(topLeft.getX(), topLeft.getY(), size.getX(), size.getY(), startAngle, arcAngle);
    }

    private void fillRect(double left, double top, double width, double height) {
        Point2I topLeft = toCanvasPosition(left, top);
        Point2I size = toCanvasOffset(width, height);

        graphics.fillRect(topLeft.getX(), topLeft.getY(), size.getX(), size.getY());
    }

    private void drawRect(double left, double top, double width, double height) {
        Point2I topLeft = toCanvasPosition(left, top);
        Point2I size = toCanvasOffset(width, height);

        graphics.drawRect(topLeft.getX(), topLeft.getY(), size.getX(), size.getY());
    }

    private void drawPolygon(Point2D... points) {
        int pointCount = points.length;

        for (int pointIndex = 1; pointIndex < pointCount; ++pointIndex) {
            Point2D pointA = points[pointIndex];
            Point2D pointB = points[pointIndex - 1];
            drawLine(pointA.getX(), pointA.getY(), pointB.getX(), pointB.getY());
        }

        Point2D pointA = points[0];
        Point2D pointB = points[pointCount - 1];
        drawLine(pointA.getX(), pointA.getY(), pointB.getX(), pointB.getY());
    }

    private Point2I toCanvasOffset(double x, double y) {
        return new Point2I(x * canvasWidth / width, y * canvasHeight / height);
    }

    private Point2I toCanvasPosition(double x, double y) {
        return new Point2I((x - left) * canvasWidth / width, (y - top) * canvasHeight / height);
    }

    private static final class Point2I {

        private int x;
        private int y;

        private Point2I(double x, double y) {
            this.x = toInt(round(x));
            this.y = toInt(round(y));
        }

        private Point2I(int x, int y) {
            this.x = x;
            this.y = y;
        }

        private Point2I() {
        }

        public int getX() {
            return x;
        }

        public void setX(int x) {
            this.x = x;
        }

        public int getY() {
            return y;
        }

        public void setY(int y) {
            this.y = y;
        }

        private static int toInt(double value) {
            @SuppressWarnings("NumericCastThatLosesPrecision")
            int intValue = (int) value;
            if (abs((double) intValue - value) < 1.0D) {
                return intValue;
            }
            throw new IllegalArgumentException("Can't convert double " + value + " to int.");
        }
    }

    private static final class Point2D {

        private double x;
        private double y;

        private Point2D(double x, double y) {
            this.x = x;
            this.y = y;
        }

        private Point2D() {
        }

        public double getX() {
            return x;
        }

        public void setX(double x) {
            this.x = x;
        }

        public double getY() {
            return y;
        }

        public void setY(double y) {
            this.y = y;
        }
    }

    class LinePoints {

        public Point2D start;
        public Point2D end;

        LinePoints(double x, double y, double x1, double y1) {
            start = new Point2D(x, y);
            end = new Point2D(x1, y1);
        }

    }

    class Player extends Thread {

        Player() {

            try {
                listener = new ServerSocket(8901);
            } catch (Exception e) {
            }

        }

        public void run() {
            while (true) {
                if (listener != null) {
                    try {
                        if (socket == null) {
                            socket = listener.accept();
                        }
                        if (socket != null) {
                            in = new BufferedReader(
                                    new InputStreamReader(socket.getInputStream()));
                            //out = new PrintWriter(socket.getOutputStream(), true);
                            String command = in.readLine();
                            String[] cmds = command.split(",");
                            double[] coords = new double[cmds.length];
                            for(int i = 0; i< cmds.length;i++){
                            coords[i] = Double.parseDouble(cmds[i]);
                            }
                             linePoints.add(new LinePoints(coords[0], coords[1], coords[2], coords[3]));
                                
                        }

                    } catch (IOException e) {
                    }
                }

            }

        }
    }

}
