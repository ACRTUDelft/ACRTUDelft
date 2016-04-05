package tudelft.acr.acrrcapp;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    private static MainActivity instance;
    private static WifiConnection client;

    private Button btn_forward;
    private Button btn_backward;
    private SeekBar angularSlider;

    private TextView connectionStatus;
    private TextView speed;

    private SeekBar speedSlider;


    public static MainActivity getInstance() {
        return instance;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        instance = this;
        btn_forward = (Button) findViewById(R.id.btn_forward);
        btn_backward = (Button) findViewById(R.id.btn_backward);
        angularSlider = (SeekBar) findViewById(R.id.angularSlider);

        connectionStatus = (TextView) findViewById(R.id.ConnectionStatus);
        speed = (TextView) findViewById(R.id.speed);

        speedSlider = (SeekBar) findViewById(R.id.speedSlider);

        speedSlider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                speed.setText(String.valueOf(progress / 50.f));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) { }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) { }
        });
        askIP(getString(R.string.ccEnter));
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                while(true) {
                    if(client != null && client.isReady()) {
                        float f = 0;
                        if (btn_forward.isPressed()) {
                            f += 1;
                        }
                        if (btn_backward.isPressed()) {
                            f -= 1;
                        }
                        float speed = (speedSlider.getProgress() / 50.f);
                        f *= speed;

                        float a = (angularSlider.getProgress() / 50.f) - 1f;
                        client.send(f, a);
                    }
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
        t.start();
    }

    /**
     * Ask the user to fill in either an IP address or a connect code of the server to connect to.
     * The connect code is parsed to the corresponding IP address.
     *  Calls itself when the provided connect code or IP address is of the wrong format.
     * @param msg The message to display as description.
     */
    private void askIP(String msg) {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle(R.string.createConnection);
        builder.setMessage(msg);
        final EditText input = new EditText(this);
        builder.setView(input);
        builder.setPositiveButton(R.string.ok, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                try {
                    String IP;
                    String inputText = input.getText().toString();
                    if (inputText.indexOf('.') == -1) {    // If 4-digit connect code
                        IP = WifiConnection.parseConnectCode(inputText);
                    } else {
                        if (inputText.split("\\.").length != 4) {
                            throw new IllegalArgumentException(getString(R.string.IPsyntaxError));
                        }
                        IP = inputText;
                    }
                    connectionStatus.setText(getString(R.string.connectingTo, IP));
                    client = new WifiConnection(IP);
                } catch (IllegalArgumentException e) {
                    askIP(e.getMessage());
                }
            }
        });
        builder.setNegativeButton(R.string.cancel, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
            }
        });

        builder.show();
    }

    /**
     * Close the current connection and connect to the same ip again.
     * @param view the view that called this method.
     */
    public void reconnect(View view) {
        if(client != null) {
            client.close();
            String ip = client.getServerIP();
            client = new WifiConnection(ip);
        }
    }

    /**
     * Close the current connection and connect to a new one.
     * @param view the view that called this method.
     */
    public void newConnect(View view) {
        connectionStatus.setText(getString(R.string.notConnected));
        if(client != null) client.close();
        askIP(getString(R.string.ccEnter));
    }
}

