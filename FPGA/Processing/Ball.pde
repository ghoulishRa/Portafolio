public class Ball{
  private float x, Vx;
  private float y, Vy;
  private float d;
  private boolean canMove;
  private String edge;
    
  public Ball(){
    x = width/2;
    d = 28;  
    y = height - 60 - d/2;
    Vx = random(-10,10);
    Vy = -15;

  }
  
  public void display(int move){
    if(canMove){
      x += Vx;
      y += Vy;
      //checkWalls
      if(x < 5){
        Vx *= -1;
        x = 5;
      }      
      if(x > width-5){
        Vx *= -1;
        x = width-5;
      }
      if(y < 5){
        Vy *= -1;
      } else if (y > height - d/2){
        canMove = false;
        y = height - 70;
        Vy = -5;
        lives--;
      }
    } else {
      if(move > 140 && move < 150){
      x = 1750;
    }else if(move > 120 && move < 130){
      x = 1500;
    }else if(move > 100 && move < 110){
      x = 1250;
    }else if(move > 80 && move < 90){
      x = 1000;
    }else if(move > 60 && move < 70){
      x = 750;
    }else if(move > 40 && move < 50){
      x = 500;
    }else if(move > 20 && move < 30){
      x = 250;
    }else if(move > 0 && move < 10){
      x = 0;
    }
    }
    fill(155);
    ellipse(x,y,d+10,d+10);    
    fill(200,170,0);
    ellipse(x,y,d,d);
  }
  
  public void checkPaddle(Paddle pad, int move){
      if(x > pad.x && x < pad.x + pad.w && y > pad.y - d/2 && y < pad.y+2){
        //Vy *= -1;
        Vx += (x - move)/10;
       //CAP THE VX
        if (Vx > 8){
          Vx = 8;
        }
        if (Vx < -8){
          Vx = -8;
        }
        if(Vx < 0){
          Vy = -12 - Vx;
        } else {
          Vy = -12 + Vx;
        }
      } 

  }
}
