package nordic.android.nrf;



/**
 * Interface {@link TemplateManagerCallbacks} must be implemented by {@link TemplateActivity} in order to receive callbacks from {@link TemplateManager}
 */
public interface TemplateManagerCallbacks extends BleManagerCallbacks {

	// TODO add more callbacks. Callbacks are called when a data has been received/written to a remote device. This is the way how the manager notifies the activity about this event.

	/**
	 * Called when a value is received.
	 * 
	 * @param value
	 *            the new value
	 */
	public void onSampleValueReceived(int value);

}
