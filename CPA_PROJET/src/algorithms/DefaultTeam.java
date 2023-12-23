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
import java.util.Random;
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
      RefreshPoint();
      cercle = measureExecutionTime(0, (Supplier<Circle>) () -> CercleMin(points));
      cercle = measureExecutionTime(1, (Supplier<Circle>) () -> WelzlAlgorithme(points, new ArrayList<Point>()));
      RefreshPoint();
    }

    if (cercle == null) {
      cercle = new Circle(new Point(0, 0), 0);
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

    for (Line pointA : antipodales)
      if (pointA.getP().distance(pointA.getQ()) > p.distance(q)) {
        p = pointA.getP();
        q = pointA.getQ();
      }

    return new Line(p, q);
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
          // System.out.println("Turn " + turn + " / Number " + i + " : " + angle);
          enveloppe.remove(i);
          toQuit = true;
          break;
        }
      }

      if (!toQuit) {
        estConvexe = true;
      }
    }

    // System.out.println(enveloppe.toString());
    return enveloppe;
  }

  // Fonction d'appel de la fonction de calcul de base du cercle minimum
  private Circle CercleMinOptimise(ArrayList<Point> points) {

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

  // Fonction d'appel de l'algorithme de Welzl
  public Circle WelzlAlgorithme(ArrayList<Point> P, ArrayList<Point> R) {
    ArrayList<Point> P1 = new ArrayList<Point>(P);
    Random rand = new Random(); // point au hasard
    Circle d = new Circle(new Point(0, 0), 0);
    if (P1.isEmpty() || R.size() == 3) {
      d = WelzlMinimumCircle(new ArrayList<Point>(), R);
    } else {
      Point pt = P1.get(rand.nextInt(P1.size()));
      P1.remove(pt);
      d = WelzlAlgorithme(P1, R);
      if (d != null && !circleContains(d, pt)) {
        R.add(pt);
        d = WelzlAlgorithme(P1, R);
        R.remove(pt);
      }
    }
    return d;
  }

  private Circle CercleMin(ArrayList<Point> inputPoints){
    ArrayList<Point> points = (ArrayList<Point>) inputPoints.clone();
    if (points.size()<1) return null;
    double cX,cY,cRadius,cRadiusSquared;
    for (Point p: points){
        for (Point q: points){
            cX = .5*(p.x+q.x);
            cY = .5*(p.y+q.y);
            cRadiusSquared = 0.25*((p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y));
            boolean allHit = true;
            for (Point s: points)
                if ((s.x-cX)*(s.x-cX)+(s.y-cY)*(s.y-cY)>cRadiusSquared){
                    allHit = false;
                    break;
                }
            if (allHit) return new Circle(new Point((int)cX,(int)cY),(int)Math.sqrt(cRadiusSquared));
        }
    }
    double resX=0;
    double resY=0;
    double resRadiusSquared=Double.MAX_VALUE;
    for (int i=0;i<points.size();i++){
        for (int j=i+1;j<points.size();j++){
            for (int k=j+1;k<points.size();k++){
                Point p=points.get(i);
                Point q=points.get(j);
                Point r=points.get(k);
                //si les trois sont colineaires on passe
                if ((q.x-p.x)*(r.y-p.y)-(q.y-p.y)*(r.x-p.x)==0) continue;
                //si p et q sont sur la meme ligne, ou p et r sont sur la meme ligne, on les echange
                if ((p.y==q.y)||(p.y==r.y)) {
                    if (p.y==q.y){
                        p=points.get(k); //ici on est certain que p n'est sur la meme ligne de ni q ni r
                        r=points.get(i); //parce que les trois points sont non-colineaires
                    } else {
                        p=points.get(j); //ici on est certain que p n'est sur la meme ligne de ni q ni r
                        q=points.get(i); //parce que les trois points sont non-colineaires
                    }
                }
                //on cherche les coordonnees du cercle circonscrit du triangle pqr
                //soit m=(p+q)/2 et n=(p+r)/2
                double mX=.5*(p.x+q.x);
                double mY=.5*(p.y+q.y);
                double nX=.5*(p.x+r.x);
                double nY=.5*(p.y+r.y);
                //soit y=alpha1*x+beta1 l'equation de la droite passant par m et perpendiculaire a la droite (pq)
                //soit y=alpha2*x+beta2 l'equation de la droite passant par n et perpendiculaire a la droite (pr)
                double alpha1=(q.x-p.x)/(double)(p.y-q.y);
                double beta1=mY-alpha1*mX;
                double alpha2=(r.x-p.x)/(double)(p.y-r.y);
                double beta2=nY-alpha2*nX;
                //le centre c du cercle est alors le point d'intersection des deux droites ci-dessus
                cX=(beta2-beta1)/(double)(alpha1-alpha2);
                cY=alpha1*cX+beta1;
                cRadiusSquared=(p.x-cX)*(p.x-cX)+(p.y-cY)*(p.y-cY);
                if (cRadiusSquared>=resRadiusSquared) continue;
                boolean allHit = true;
                for (Point s: points)
                    if ((s.x-cX)*(s.x-cX)+(s.y-cY)*(s.y-cY)>cRadiusSquared){
                        allHit = false;
                        break;
                    }
                if (allHit) {System.out.println("Found r="+Math.sqrt(cRadiusSquared));resX=cX;resY=cY;resRadiusSquared=cRadiusSquared;}
            }
        }
    }
    return new Circle(new Point((int)resX,(int)resY),(int)Math.sqrt(resRadiusSquared));
}


  /***************************************
   * ALGORITHME DE WELZL - CERCLE MINIMUM
   ****************************************/

  // Calcul du cercle minimum pour l'algorithme de Welzl
  public Circle WelzlMinimumCircle(ArrayList<Point> P, ArrayList<Point> R) {
    if (P.isEmpty() && R.size() == 0)
      return new Circle(new Point(0, 0), 0);
    Circle D = new Circle(new Point(0, 0), 0);
    if (R.size() == 1) {
      D = new Circle(R.get(0), 0);
    }
    if (R.size() == 2) {
      double cx = (R.get(0).x + R.get(1).x) / 2;
      double cy = (R.get(0).y + R.get(1).y) / 2;
      double d = R.get(0).distance(R.get(1)) / 2;
      Point p = new Point((int) cx, (int) cy);
      D = new Circle(p, (int) Math.ceil(d));
    } else {
      if (R.size() == 3)
        D = WelzlCircle3Points(R.get(0), R.get(1), R.get(2));
    }
    return D;
  }


  // Calcul du cercle minimum avec 3 points pour l'algorithme de Welzl
  private Circle WelzlCircle3Points(Point pointA, Point pointB, Point pointC) {
    double d = (pointA.x * (pointB.y - pointC.y) + pointB.x * (pointC.y - pointA.y) + pointC.x * (pointA.y - pointB.y))
        * 2;
    if (d == 0)
      return new Circle(new Point(0, 0), 0);

    double x = ((calculNormeVecteur(pointA) * (pointB.y - pointC.y))
        + (calculNormeVecteur(pointB) * (pointC.y - pointA.y))
        + (calculNormeVecteur(pointC) * (pointA.y - pointB.y))) / d;

    double y = ((calculNormeVecteur(pointA) * (pointC.x - pointB.x))
        + (calculNormeVecteur(pointB) * (pointA.x - pointC.x))
        + (calculNormeVecteur(pointC) * (pointB.x - pointA.x))) / d;

    Point p = new Point((int) x, (int) y);

    return new Circle(p, (int) Math.ceil(p.distance(pointA)));
  }

  /**************************************
   * FONCTIONS DE CALCUL AVEC DES POINTS
   ***************************************/

  // Calcul du produit vectoriel [pq] [st]
  private double calculProduitVectoriel(Point p, Point q, Point s, Point t) {
    return ((q.x - p.x) * (t.y - s.y) - (q.y - p.y) * (t.x - s.x));
  }

  // Calcul de l'angle [ab] [bc]
  private static double calculerAngle(Point pointA, Point pointB, Point pointC) {

    double angleRad = Math.atan2(pointC.getY() - pointB.getY(), pointC.getX() - pointB.getX())
        - Math.atan2(pointA.getY() - pointB.getY(), pointA.getX() - pointB.getX());
    angleRad = (angleRad + 2 * Math.PI) % (2 * Math.PI);

    return Math.toDegrees(angleRad);
  }

  // Calcul de la distance entre deux points
  private double distanceBetween2Points(Point p1, Point p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
  }

  // Calcul de la distance entre trois points
  private double distanceBetween3Points(Point p, Point pointA, Point pointB) {
    return Math.abs(calculProduitVectoriel(pointA, pointB, pointA, p));
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

  // Calcul de la norme d'un vecteur
  private int calculNormeVecteur(Point pointA) {
    return (int) (Math.pow(pointA.x, 2) + Math.pow(pointA.y, 2));
  }

  /****************************************************************************
   * FONCTIONS POUR MESURER LE TEMPS D'EXECUTION ET POUR STOCKER LES RÉSULTATS
   *****************************************************************************/

  // Mesurer le temps d'execution de la fonction
  private static <T> T measureExecutionTime(int AlgoType, Supplier<T> function) {
    long startTime = System.nanoTime() / 1000; // Convertir en microsecondes
    T result = function.get();
    long endTime = System.nanoTime() / 1000; // Convertir en microsecondes
    long duration = endTime - startTime;
    System.out.println("Execution time: " + duration);
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
      FileName = "times_default.txt";
    } else if (AlgoType == 1) { // Algo de Welzl
      FileName = "times_welzl.txt";
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
