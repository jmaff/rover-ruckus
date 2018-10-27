package com.ftc12835.library.vision;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;
import android.widget.LinearLayout;

public class DrawViewSource extends View {

    private Paint paint;
    private Resources resources;
    private Bitmap bitmap;

    public DrawViewSource(Context context) {
        super(context);
        init(context, null, 0);
    }

    public DrawViewSource(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context, attrs, 0);
    }

    public DrawViewSource(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init(context, attrs, defStyleAttr);
    }

    private void init(Context context, AttributeSet attrs, int defStyleAttr) {
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        resources = context.getResources();
        setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.MATCH_PARENT));
    }

    @SuppressLint("NewApi")
    public DrawViewSource(Context context, AttributeSet attrs,
                          int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
    }

    public void onFrame(Bitmap map){
        bitmap = map;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        canvas.drawColor(0, android.graphics.PorterDuff.Mode.CLEAR);
        int deviceOrientation = getContext().getResources().getConfiguration().orientation;

        if(bitmap != null){
            canvas.drawBitmap(bitmap, 0, 0,null);
        }
    }
}
