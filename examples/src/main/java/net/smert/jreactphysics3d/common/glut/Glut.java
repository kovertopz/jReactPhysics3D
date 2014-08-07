package net.smert.jreactphysics3d.common.glut;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import org.lwjgl.LWJGLException;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.lwjgl.opengl.PixelFormat;

/**
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Glut {

    public static int GLUT_ACTIVE_ALT = 2;
    public static int GLUT_DOWN = 1;
    public static int GLUT_UP = 0;

    public static int GLUT_LEFT_BUTTON = 0;
    public static int GLUT_MIDDLE_BUTTON = 2;
    public static int GLUT_RIGHT_BUTTON = 1;
    public static int GLUT_WHEEL_UP = 3;
    public static int GLUT_WHEEL_DOWN = 4;

    public static int GLUT_DEPTH = 1;
    public static int GLUT_DOUBLE = 2;
    public static int GLUT_RGBA = 4;

    public static int GL_MULTISAMPLE = 8;

    public static boolean isRunning = true;
    public static boolean renderAttempt = false;
    public static DisplayMode displayMode;
    public static int windowPositionX;
    public static int windowPositionY;
    public static int windowSizeX;
    public static int windowSizeY;
    public static Method closeMethod;
    public static Object closeObject;
    public static Method displayMethod;
    public static Object displayObject;
    public static Method idleMethod;
    public static Object idleObject;
    public static Method keyboardMethod;
    public static Object keyboardObject;
    public static Method mouseMethod;
    public static Object mouseObject;
    public static Method motionMethod;
    public static Object motionObject;
    public static Method reshapeMethod;
    public static Object reshapeObject;
    public static PixelFormat pixelFormat;

    public static void glutCloseFunc(Object object, String closeFunc) {
        Class cls = object.getClass();
        Class[] noParams = {};
        try {
            closeMethod = cls.getDeclaredMethod(closeFunc, noParams);
            closeObject = object;
        } catch (NoSuchMethodException | SecurityException ex) {
            System.out.println("The class '" + object.getClass() + "' must have a method '" + closeFunc + "' that takes 0 parameters.");
            ex.printStackTrace();
            System.exit(1);
        }
    }

    public static void glutCreateWindow(String windowsTitle) {
        try {
            DisplayMode[] dms = Display.getAvailableDisplayModes();
            for (DisplayMode dm : dms) {
                if ((dm.getWidth() == windowSizeX) && (dm.getHeight() == windowSizeY) && (dm.getBitsPerPixel() == 24)) {
                    displayMode = dm;
                    break;
                }
            }

            if (displayMode == null) {
                displayMode = new DisplayMode(800, 600);
            }

            Display.setDisplayMode(displayMode);
            Display.setTitle(windowsTitle);
            Display.create();
            Display.setLocation(windowPositionX, windowPositionY);
            Display.setResizable(true);
        } catch (LWJGLException ex) {
            ex.printStackTrace();
            System.exit(1);
        }
    }

    public static void glutDisplayFunc(Object object, String displayFunc) {
        Class cls = object.getClass();
        Class[] noParams = {};
        try {
            displayMethod = cls.getDeclaredMethod(displayFunc, noParams);
            displayObject = object;
        } catch (NoSuchMethodException | SecurityException ex) {
            System.out.println("The class '" + object.getClass() + "' must have a method '" + displayFunc + "' that takes 0 parameters.");
            ex.printStackTrace();
            System.exit(1);
        }
    }

    public static int glutGetModifiers() {
        if (Keyboard.isKeyDown(Keyboard.KEY_LMENU) || Keyboard.isKeyDown(Keyboard.KEY_RMENU)) {
            return GLUT_ACTIVE_ALT;
        }
        return 0;
    }

    public static void glutIdleFunc(Object object, String idleFunc) {
        Class cls = object.getClass();
        Class[] noParams = {};
        try {
            idleMethod = cls.getDeclaredMethod(idleFunc, noParams);
            idleObject = object;
        } catch (NoSuchMethodException | SecurityException ex) {
            System.out.println("The class '" + object.getClass() + "' must have a method '" + idleFunc + "' that takes 0 parameters.");
            ex.printStackTrace();
            System.exit(1);
        }
    }

    public static void glutInit(String[] args) {
    }

    public static void glutInitDisplayMode(int displayMode) {
        pixelFormat = new PixelFormat().withBitsPerPixel(24);

        if ((displayMode & GLUT_DEPTH) != 0) {
            pixelFormat.withDepthBits(24).withStencilBits(8);
        }
        if ((displayMode & GL_MULTISAMPLE) != 0) {
            pixelFormat.withSamples(4);
        }
    }

    public static void glutInitWindowPosition(int x, int y) {
        windowPositionX = x;
        windowPositionY = y;
    }

    public static void glutInitWindowSize(int x, int y) {
        windowSizeX = x;
        windowSizeY = y;
    }

    public static void glutLeaveMainLoop() {
        isRunning = false;
    }

    public static void glutKeyboardFunc(Object object, String keyboardFunc) {
        Class cls = object.getClass();
        Class[] charAndIntAndInt = {Integer.TYPE, Integer.TYPE, Integer.TYPE};
        try {
            keyboardMethod = cls.getDeclaredMethod(keyboardFunc, charAndIntAndInt);
            keyboardObject = object;
        } catch (NoSuchMethodException | SecurityException ex) {
            System.out.println("The class '" + object.getClass() + "' must have a method '" + keyboardFunc + "' that takes 3 parameters. (int, int, int)");
            ex.printStackTrace();
            System.exit(1);
        }
    }

    public static void glutMainLoop() {
        try {
            while (isRunning) {
                if (Display.isCloseRequested() == true) {
                    glutLeaveMainLoop();
                }

                if (Display.wasResized()) {
                    int windowSizeX = Display.getWidth();
                    int windowSizeY = Display.getHeight();
                    reshapeMethod.invoke(reshapeObject, windowSizeX, windowSizeY);
                }

                int mouseX = Mouse.getX();
                int mouseY = Mouse.getY();

                while (Keyboard.next()) {
                    keyboardMethod.invoke(keyboardObject, Keyboard.getEventKey(), mouseX, mouseY);
                }

                while (Mouse.next() == true) {
                    int mouseButton = Mouse.getEventButton();
                    if (mouseButton != -1) {
                        int mouseState = Mouse.getEventButtonState() ? 1 : 0;
                        mouseMethod.invoke(mouseObject, mouseButton, mouseState, mouseX, mouseY);
                    } else if (Mouse.getEventDWheel() != 0) {
                        mouseButton = Mouse.getEventDWheel() > 0 ? 3 : 4;
                        mouseMethod.invoke(mouseObject, mouseButton, 0, mouseX, mouseY);
                    } else {
                        motionMethod.invoke(motionObject, mouseX, mouseY);
                    }
                }

                idleMethod.invoke(idleObject);

                if (renderAttempt) {
                    displayMethod.invoke(displayObject);
                }

                renderAttempt = false;

                Display.update();
            }

            closeMethod.invoke(closeObject);
        } catch (IllegalAccessException | IllegalArgumentException ex) {
            ex.printStackTrace();
            System.exit(1);
        } catch (InvocationTargetException ex) {
            ex.getCause().printStackTrace();
            System.exit(1);
        }
    }

    public static void glutMouseFunc(Object object, String mouseFunc) {
        Class cls = object.getClass();
        Class[] intAndIntAndIntAndInt = {Integer.TYPE, Integer.TYPE, Integer.TYPE, Integer.TYPE};
        try {
            mouseMethod = cls.getDeclaredMethod(mouseFunc, intAndIntAndIntAndInt);
            mouseObject = object;
        } catch (NoSuchMethodException | SecurityException ex) {
            System.out.println("The class '" + object.getClass() + "' must have a method '" + mouseFunc + "' that takes 4 parameters. (int, int, int, int)");
            ex.printStackTrace();
            System.exit(1);
        }
    }

    public static void glutMotionFunc(Object object, String motionFunc) {
        Class cls = object.getClass();
        Class[] intAndInt = {Integer.TYPE, Integer.TYPE};
        try {
            motionMethod = cls.getDeclaredMethod(motionFunc, intAndInt);
            motionObject = object;
        } catch (NoSuchMethodException | SecurityException ex) {
            System.out.println("The class '" + object.getClass() + "' must have a method '" + motionFunc + "' that takes 2 parameters. (int, int)");
            ex.printStackTrace();
            System.exit(1);
        }
    }

    public static void glutPostRedisplay() {
        renderAttempt = true;
    }

    public static void glutReshapeFunc(Object object, String reshapeFunc) {
        Class cls = object.getClass();
        Class[] intAndInt = {Integer.TYPE, Integer.TYPE};
        try {
            reshapeMethod = cls.getDeclaredMethod(reshapeFunc, intAndInt);
            reshapeObject = object;
        } catch (NoSuchMethodException | SecurityException ex) {
            System.out.println("The class '" + object.getClass() + "' must have a method '" + reshapeFunc + "' that takes 2 parameters. (int, int)");
            ex.printStackTrace();
            System.exit(1);
        }
    }

    public static void glutSwapBuffers() {
    }

}
