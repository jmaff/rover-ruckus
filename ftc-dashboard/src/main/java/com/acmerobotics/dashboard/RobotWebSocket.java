package com.acmerobotics.dashboard;

import android.util.Log;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageDeserializer;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;
import fi.iki.elonen.NanoWSD;

/**
 * WebSocket connection to a dashboard client.
 */
public class RobotWebSocket extends NanoWSD.WebSocket {
    public static final boolean DEBUG = false;

    private static final Gson GSON = new GsonBuilder()
            .registerTypeAdapter(Message.class, new MessageDeserializer())
            .create();

    private RobotDashboard dashboard;

    RobotWebSocket(NanoHTTPD.IHTTPSession handshakeRequest, RobotDashboard dash) {
        super(handshakeRequest);
        dashboard = dash;
    }

    @Override
    protected void onOpen() {
        if (DEBUG) Log.i(RobotDashboard.TAG, "[OPEN]\t" + this.getHandshakeRequest().getRemoteIpAddress());
        dashboard.addSocket(this);
    }

    @Override
    protected void onClose(NanoWSD.WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) {
        if (DEBUG) Log.i(RobotDashboard.TAG, "[CLOSE]\t" + this.getHandshakeRequest().getRemoteIpAddress());
        dashboard.removeSocket(this);
    }

    @Override
    protected void onMessage(NanoWSD.WebSocketFrame message) {
        Message msg = GSON.fromJson(message.getTextPayload(), Message.class);
        if (DEBUG) Log.i(RobotDashboard.TAG, "[RECV]\t" + message.getTextPayload());
        dashboard.onMessage(this, msg);
    }

    @Override
    protected void onPong(NanoWSD.WebSocketFrame pong) {

    }

    @Override
    protected void onException(IOException exception) {

    }

    /**
     * Sends a message to the connected client.
     */
    public void send(Message message) {
        try {
            String messageStr = GSON.toJson(message);
            if (DEBUG) Log.i(RobotDashboard.TAG, "[SENT]\t" + messageStr);
            send(messageStr);
        } catch (IOException e) {
            Log.w(RobotDashboard.TAG, e);
        }
    }

}
