package nordic.android.nrf;

import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.Context;

import java.util.LinkedList;
import java.util.Queue;
import java.util.UUID;

import no.nordicsemi.android.log.Logger;
;

/**
 * Modify to template manager to match your requirements.
 */
public class TemplateManager extends BleManager<TemplateManagerCallbacks> {
	private static final String TAG = "TemplateManager";

	/** The service UUID */
	public final static UUID SERVICE_UUID = UUID.fromString("0000180D-0000-1000-8000-00805f9b34fb"); // TODO change the UUID to your match your service
	/** The characteristic UUID */
	private static final UUID MEASUREMENT_CHARACTERISTIC_UUID = UUID.fromString("00002A37-0000-1000-8000-00805f9b34fb"); // TODO change the UUID to your match your characteristic

	// TODO add more services and characteristics, if required
	private BluetoothGattCharacteristic mCharacteristic;

	public TemplateManager(final Context context) {
		super(context);
	}

	@Override
	protected BleManagerGattCallback getGattCallback() {
		return mGattCallback;
	}

	/**
	 * BluetoothGatt callbacks for connection/disconnection, service discovery, receiving indication, etc
	 */
	private final BleManagerGattCallback mGattCallback = new BleManagerGattCallback() {

		@Override
		protected Queue<Request> initGatt(final BluetoothGatt gatt) {
			final LinkedList<Request> requests = new LinkedList<>();
			// TODO initialize your device, enable required notifications and indications, write what needs to be written to start working
			requests.push(Request.newEnableNotificationsRequest(mCharacteristic));
			return requests;
		}

		@Override
		protected boolean isRequiredServiceSupported(final BluetoothGatt gatt) {
			final BluetoothGattService service = gatt.getService(SERVICE_UUID);
			if (service != null) {
				mCharacteristic = service.getCharacteristic(MEASUREMENT_CHARACTERISTIC_UUID);
			}
			return mCharacteristic != null;
		}

		@Override
		protected void onDeviceDisconnected() {
			mCharacteristic = null;
		}

		// TODO implement data handlers. Methods below are called after the initialization is complete.

		@Override
		protected void onDeviceReady() {
			super.onDeviceReady();

			// TODO initialization is now ready. The activity is being notified using TemplateManagerCallbacks#onDeviceReady() method.
			// This method may be removed from this class if not required as the super class implementation handles this event.
		}

		@Override
		protected void onCharacteristicNotified(final BluetoothGatt gatt, final BluetoothGattCharacteristic characteristic) {
			// TODO this method is called when a notification has been received
			// This method may be removed from this class if not required

			if (mLogSession != null)
				Logger.a(mLogSession, TemplateParser.parse(characteristic));

			int value;
			final int flags = characteristic.getIntValue(BluetoothGattCharacteristic.FORMAT_UINT8, 0);
			if ((flags & 0x01) > 0) {
				value = characteristic.getIntValue(BluetoothGattCharacteristic.FORMAT_UINT16, 1);
			} else {
				value = characteristic.getIntValue(BluetoothGattCharacteristic.FORMAT_UINT8, 1);
			}
			//This will send callback to the Activity when new value is received from HR device
			mCallbacks.onSampleValueReceived(value);
		}

		@Override
		protected void onCharacteristicIndicated(final BluetoothGatt gatt, final BluetoothGattCharacteristic characteristic) {
			// TODO this method is called when an indication has been received
			// This method may be removed from this class if not required
		}

		@Override
		protected void onCharacteristicRead(final BluetoothGatt gatt, final BluetoothGattCharacteristic characteristic) {
			// TODO this method is called when the characteristic has been read
			// This method may be removed from this class if not required
		}

		@Override
		protected void onCharacteristicWrite(final BluetoothGatt gatt, final BluetoothGattCharacteristic characteristic) {
			// TODO this method is called when the characteristic has been written
			// This method may be removed from this class if not required
		}
	};


}
