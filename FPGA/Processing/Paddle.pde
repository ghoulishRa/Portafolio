public class Paddle{
   //Variables (x,y, width, height (rectangle))
   private float x, y, w, h;
   
   public Paddle(){
     x = width/2;
     y = height - 60;
     w = 250;
     h = 20;
   }
   
   public void display(int move){
    
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

     fill(255,255,0); //
     rect(x-3,y-3,w+6,h+6,8);     
     fill(0,50,100);
     rect(x,y,w,h,8); 
     stroke(155,0,55);
     strokeWeight(3);     
     line(x+3,y+h/2,x+w-3,y+h/2);
     line(x+w/3, y+2, x+w/3, y+h-2);
     line(x+2*w/3, y+2, x+2*w/3, y+h-2);     
   }
  
  
}
