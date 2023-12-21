package algorithms;

import java.awt.AWTException;
import java.awt.Color;
import java.awt.Point;
import java.awt.Robot;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Stack;
import java.util.function.Supplier;

import supportGUI.Circle;
import supportGUI.Line;

public class DefaultTeam {

  // Nombre de test pour chaque algorithme de calcul de cercle minimum
  private int nombreDeTest = 2000;

  /********************************
   * FONCTIONS EXECUTÉE AU CLAVIER
   ********************************/

  // Fonction executée au clavier avec "D"
  public Line calculDiametre(ArrayList<Point> points) {
    return Diametre(points);
  }

  // Fonction executée au clavier avec "O"
  public Line calculDiametreOptimise(ArrayList<Point> points) {
    return DiametreOptimise(points);
  }

  // Fonction executée au clavier avec "C"
  public Circle calculCercleMin(ArrayList<Point> points) {
    Circle cercle = new Circle(new Point(0, 0), 0);

    deleteFileContent(0);
    deleteFileContent(1);

    for (int i = 0; i < nombreDeTest; i++) {
      cercle = measureExecutionTime(0, (Supplier<Circle>) () -> CercleMin(points));
      RefreshPoint();
    }

    for (int i = 0; i < nombreDeTest; i++) {
      cercle = measureExecutionTime(1, (Supplier<Circle>) () -> WelzlAlgorithme(points));
      RefreshPoint();
    }

    return cercle;
  }

  // Fonction executée au clavier avec "E"
  public ArrayList<Point> enveloppeConvexe(ArrayList<Point> points) {
    return GrahamAlgorithme(points);
  }

  /**********************
   * FONCTIONS D'APPELS
   *********************/

  // Fonction d'appel de la fonction de calcul de diamètre
  private Line Diametre(ArrayList<Point> points) {
    // Vérifie que la liste de points est valide et contient au moins deux points.
    if (points == null || points.size() < 2) {
      throw new IllegalArgumentException("La liste de points doit contenir au moins deux points.");
    }

    // Initialise les variables avec les deux premiers points de la liste.
    Point premierPoint = points.get(0);
    Point deuxiemePoint = points.get(1);
    double distanceMaximale = distanceBetween2Points(premierPoint, deuxiemePoint);

    // Parcours toutes les combinaisons possibles de points.
    for (int i = 0; i < points.size() - 1; i++) {
      Point pointCourant = points.get(i);

      for (int j = i + 1; j < points.size(); j++) {
        // Calcule la distance entre le point actuel et le point courant.
        double distance = distanceBetween2Points(pointCourant, points.get(j));

        // Met à jour les points et la distance maximale.
        if (distance > distanceMaximale) {
          distanceMaximale = distance;
          premierPoint = pointCourant;
          deuxiemePoint = points.get(j);
        }
      }
    }

    return new Line(premierPoint, deuxiemePoint);
  }

  // Fonction d'appel de la fonction de calcul de diamètre optimisé
  private Line DiametreOptimise(ArrayList<Point> points) {
    // Vérifie que la liste de points est valide et contient au moins deux points.
    if (points == null || points.size() < 2) {
      throw new IllegalArgumentException("La liste de points doit contenir au moins deux points.");
    }

    ArrayList<Line> antipodales = calculPointsOpposes(points);

    Point p = antipodales.get(0).getP();
    Point q = antipodales.get(0).getQ();

    for (Line a : antipodales)
      if (a.getP().distance(a.getQ()) > p.distance(q)) {
        p = a.getP();
        q = a.getQ();
      }

    return new Line(p, q);
  }

  // Fonction d'appel de la fonction de calcul de base du cercle minimum
  private Circle CercleMin(ArrayList<Point> points) {

    if (points.size() < 1)
      return null;

    ArrayList<Point> rest = (ArrayList<Point>) points.clone();
    Point dummy = rest.get(0);
    Point p = dummy;
    for (Point s : rest)
      if (dummy.distance(s) > dummy.distance(p))
        p = s;
    Point q = p;
    for (Point s : rest)
      if (p.distance(s) > p.distance(q))
        q = s;
    double cX = .5 * (p.x + q.x);
    double cY = .5 * (p.y + q.y);
    double cRadius = .5 * p.distance(q);
    rest.remove(p);
    rest.remove(q);
    while (!rest.isEmpty()) {
      Point s = rest.remove(0);
      double distanceFromCToS = Math.sqrt((s.x - cX) * (s.x - cX) + (s.y - cY) * (s.y - cY));
      if (distanceFromCToS <= cRadius)
        continue;
      double cPrimeRadius = .5 * (cRadius + distanceFromCToS);
      double alpha = cPrimeRadius / (double) (distanceFromCToS);
      double beta = (distanceFromCToS - cPrimeRadius) / (double) (distanceFromCToS);
      double cPrimeX = alpha * cX + beta * s.x;
      double cPrimeY = alpha * cY + beta * s.y;
      cRadius = cPrimeRadius;
      cX = cPrimeX;
      cY = cPrimeY;
    }

    return new Circle(new Point((int) cX, (int) cY), (int) cRadius);
  }

  // Fonction d'appel de l'algorithme de Graham
  private ArrayList<Point> GrahamAlgorithme(ArrayList<Point> points) {

    if (points.size() < 4)
      return points;

    ArrayList<Point> enveloppe = new ArrayList<Point>();

    int Xmax = 0;
    for (Point point : points) {
      if (point.x > Xmax) {
        Xmax = point.x;
      }
    }

    // Récupérer les min et max Y pour chaque X
    Point[] yMin = new Point[Xmax + 1];
    Point[] yMax = new Point[Xmax + 1];

    for (Point l_point : points) {
      if (yMin[l_point.x] == null || yMin[l_point.x].y > l_point.y) {
        yMin[l_point.x] = l_point;
      }
      if (yMax[l_point.x] == null || yMax[l_point.x].y < l_point.y) {
        yMax[l_point.x] = l_point;
      }
    }

    // Ajouter les Y min dans l'enveloppe
    for (int l_yMin = 0; l_yMin < Xmax + 1; l_yMin++) {
      if (yMin[l_yMin] != null) {
        enveloppe.add(yMin[l_yMin]);
      }
    }

    // Ajouter les Y max dans l'enveloppe
    for (int l_yMax = Xmax; l_yMax > 0; l_yMax--) {
      if (yMax[l_yMax] != null) {
        enveloppe.add(yMax[l_yMax]);
      }
    }

    boolean estConvexe = false;
    int turn = 0;
    while (!estConvexe) {
      boolean toQuit = false;
      turn++;
      for (int i = 1; i < enveloppe.size() - 1; i++) {
        double angle = calculerAngle(enveloppe.get(i - 1), enveloppe.get(i), enveloppe.get(i + 1));
        if (angle < 180) {
          System.out.println("Turn " + turn + " / Number " + i + " : " + angle);
          enveloppe.remove(i);
          toQuit = true;
          break;
        }
      }

      if (!toQuit) {
        estConvexe = true;
      }
    }

    System.out.println(enveloppe.toString());
    return enveloppe;
  }

  // Fonction d'appel de l'algorithme de Welzl
  private Circle WelzlAlgorithme(ArrayList<Point> points) {
    Circle cercle = null;

    ArrayList<Point> pointsMelanges = new ArrayList<>(points);
    Collections.shuffle(pointsMelanges);

    cercle = WelzlRecursif(pointsMelanges, new ArrayList<>());

    return cercle;
  }

  /***************************************
   * ALGORITHME DE WELZL - CERCLE MINIMUM
   ****************************************/

  // Fonction récursive pour trouver le cercle minimum
  private Circle WelzlRecursif(List<Point> points, List<Point> boundary) {
    if (points.isEmpty() || boundary.size() == 3) {
      return WelzlMinimumCircle(boundary);
    }

    Point randomPoint = points.remove(0);
    Circle circle = WelzlRecursif(points, boundary);

    if (circle != null && !circleContains(circle, randomPoint)) {
      boundary.add(randomPoint);
      circle = WelzlRecursif(points, boundary);
      boundary.remove(randomPoint);
    }

    points.add(0, randomPoint);
    return circle;
  }

  // Calcul du cercle minimum pour l'algorithme de Welzl
  private static Circle WelzlMinimumCircle(List<Point> points) {
    if (points.isEmpty()) {
      return null;
    } else if (points.size() == 1) {
      // Cas avec un seul point
      return new Circle(points.get(0), 0);
    } else if (points.size() == 2) {
      // Cas avec deux points
      Point p1 = points.get(0);
      Point p2 = points.get(1);
      Point center = new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
      int radius = (int) Math.ceil(p1.distance(p2) / 2);
      return new Circle(center, radius);
    } else {
      // Autres Cas
      Point pointA = points.get(0);
      Point pointB = points.get(1);
      Point pointC = points.get(2);

      // Calcul du centre et du rayon du cercle
      double d = 2 * (pointA.x * (pointB.y - pointC.y) + pointB.x * (pointC.y - pointA.y)
          + pointC.x * (pointA.y - pointB.y));

      double ux = ((pointA.x * pointA.x + pointA.y * pointA.y) * (pointB.y - pointC.y)
          + (pointB.x * pointB.x + pointB.y * pointB.y) * (pointC.y - pointA.y)
          + (pointC.x * pointC.x + pointC.y * pointC.y) * (pointA.y - pointB.y)) / d;

      double uy = ((pointA.x * pointA.x + pointA.y * pointA.y) * (pointC.x - pointB.x)
          + (pointB.x * pointB.x + pointB.y * pointB.y) * (pointA.x - pointC.x)
          + (pointC.x * pointC.x + pointC.y * pointC.y) * (pointB.x - pointA.x)) / d;

      Point center = new Point((int) ux, (int) uy);
      int radius = (int) Math.ceil(
          Math.max(Math.max(pointA.distance(center), pointB.distance(center)), pointC.distance(center)));
      return new Circle(center, radius);
    }
  }

  /**************************************
   * FONCTIONS DE CALCUL AVEC DES POINTS
   ***************************************/

  // Calcul du produit vectoriel [pq] [st]
  private double produitVectoriel(Point p, Point q, Point s, Point t) {
    return ((q.x - p.x) * (t.y - s.y) - (q.y - p.y) * (t.x - s.x));
  }

  // Calcul de l'angle [ab] [bc]
  private static double calculerAngle(Point a, Point b, Point c) {

    double angleRad = Math.atan2(c.getY() - b.getY(), c.getX() - b.getX())
        - Math.atan2(a.getY() - b.getY(), a.getX() - b.getX());
    angleRad = (angleRad + 2 * Math.PI) % (2 * Math.PI);

    return Math.toDegrees(angleRad);
  }

  // Calcul de la distance entre deux points
  private double distanceBetween2Points(Point p1, Point p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
  }

  // Calcul de la distance entre trois points
  private double distanceBetween3Points(Point p, Point a, Point b) {
    return Math.abs(produitVectoriel(a, b, a, p));
  }

  // Calcul de deux points opposés par rapport au plan
  private ArrayList<Line> calculPointsOpposes(ArrayList<Point> points) {
    ArrayList<Point> convexList = GrahamAlgorithme(points);
    ArrayList<Line> antipodals = new ArrayList<Line>();

    int k = 1;
    while (distanceBetween3Points(convexList.get(k), convexList.get(convexList.size() - 1),
        convexList.get(0)) < distanceBetween3Points(convexList.get((k + 1) % convexList.size()),
            convexList.get(convexList.size() - 1), convexList.get(0))) {
      k++;
    }

    int i = 0;
    int j = k;

    while (i <= k && j < convexList.size()) {
      while (distanceBetween3Points(convexList.get(j), convexList.get(i),
          convexList.get(i + 1)) < distanceBetween3Points(convexList.get((j + 1) % convexList.size()),
              convexList.get(i), convexList.get(i + 1))
          && j < convexList.size() - 1) {
        antipodals.add(new Line(convexList.get(i), convexList.get(j)));
        j++;
      }
      antipodals.add(new Line(convexList.get(i), convexList.get(j)));
      i++;
    }

    return antipodals;
  }

  /****************************************************************************
   * FONCTIONS POUR MESURER LE TEMPS D'EXECUTION ET POUR STOCKER LES RÉSULTATS
   *****************************************************************************/
  
  // Mesurer le temps d'execution de la fonction
  private static <T> T measureExecutionTime(int AlgoType, Supplier<T> function) {
    long startTime = System.nanoTime();
    T result = function.get();
    long endTime = System.nanoTime();
    long duration = endTime - startTime;
    storeExecutionTime(AlgoType, duration);
    return result;
  }

  // Écrit le temps d'execution de la fonction dans le fichier
  private static void storeExecutionTime(int AlgoType, long duration) {
    String fileName = GetFileName(AlgoType);

    try (PrintWriter writer = new PrintWriter(new FileWriter(fileName, true))) {
      // Ajout du temps d'exécution dans le fichier
      writer.println(duration);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // Supprimer et recréer les fichiers
  private static void deleteFileContent(int AlgoType) {
    try {
      File file = new File(GetFileName(AlgoType));

      if (file.exists()) {
        file.delete();
        file.createNewFile();
      }

    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /********************
   * AUTRES FONCTIONS
   *********************/

  // Permet de savoir si le point est contenu dans le cercle
  private static boolean circleContains(Circle circle, Point point) {
    return circle.getCenter().distance(point) <= circle.getRadius();
  }

  // Récupérer le nom du fichier par rapport à l'Algorithme
  private static String GetFileName(int AlgoType) {
    String FileName = new String();
    if (AlgoType == 0) { // Algo de base
      FileName = "default_times.txt";
    } else if (AlgoType == 1) { // Algo de Welzl
      FileName = "welzl_times.txt";
    }
    return FileName;
  }

  // Permet de rafraichir la liste de points
  private void RefreshPoint() {
    try {

      Robot robot = new Robot();
      robot.keyPress(KeyEvent.VK_R);
      robot.keyRelease(KeyEvent.VK_R);

    } catch (AWTException e) {
      e.printStackTrace();
    }
  }

}
