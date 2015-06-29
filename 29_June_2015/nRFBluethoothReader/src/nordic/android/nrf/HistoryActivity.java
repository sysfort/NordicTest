/*
 * Copyright (c) 2015, Nordic Semiconductor
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package nordic.android.nrf;

import android.os.Bundle;
import android.util.Log;
import android.util.SparseArray;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.BaseExpandableListAdapter;
import android.widget.PopupMenu;
import android.widget.TextView;

import java.util.UUID;


// TODO The GlucoseActivity should be rewritten to use the service approach, like other do.
public class HistoryActivity extends BleProfileExpandableListActivity implements PopupMenu.OnMenuItemClickListener, HistoryManagerCallbacks {
	@SuppressWarnings("unused")
	private static final String TAG = "GlucoseActivity";

	private BaseExpandableListAdapter mAdapter;
	private HistoryManager mHistoryManager;

	private View mControlPanelStd;
	private View mControlPanelAbort;
	private TextView mUnitView;

	@Override
	protected void onCreateView(final Bundle savedInstanceState) {
		// FEATURE_INDETERMINATE_PROGRESS notifies the system, that we are going to show indeterminate progress bar in the ActionBar (during device scan)
		// requestWindowFeature(Window.FEATURE_INDETERMINATE_PROGRESS); // <- Deprecated
		setContentView(R.layout.activity_feature_history);
		setGUI();
	}

	private void setGUI() {
		mUnitView = (TextView) findViewById(R.id.unit);
		mControlPanelStd = findViewById(R.id.history_control_std);
		mControlPanelAbort = findViewById(R.id.history_control_abort);

		findViewById(R.id.action_last).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				mHistoryManager.getLastRecord();
			}
		});
		findViewById(R.id.action_all).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				mHistoryManager.getAllRecords();
				Log.i("mHistoryManager MANGER", "---=== " +mHistoryManager.getRecords());
			}
		});
		findViewById(R.id.action_abort).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				mHistoryManager.abort();
			}
		});

		// create popup menu attached to the button More
		findViewById(R.id.action_first).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				mHistoryManager.getFirstRecord();
			}
		});
		findViewById(R.id.action_refresh).setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				mHistoryManager.refreshRecords();
			}
		});
		
		Log.i("mHistoryManager MANGER", "---=== " +mHistoryManager);
//rdp
setListAdapter(mAdapter = new ExpandableRecordAdapter(this, mHistoryManager));
	}

	@Override
	protected BleManager<HistoryManagerCallbacks> initializeManager() {
		HistoryManager manager = mHistoryManager = HistoryManager.getGlucoseManager(getApplicationContext());
		manager.setGattCallbacks(this);
		Log.i("HistoryManager MANGER", "---=== " +manager);
		return manager;
	}

	@Override
	public boolean onMenuItemClick(final MenuItem item) {
		switch (item.getItemId()) {
		case R.id.action_refresh:
			Log.i("mHistoryManager MANGER", "---=== " +mHistoryManager);
			mHistoryManager.refreshRecords();
			break;
		case R.id.action_first:
			Log.i("mHistoryManager MANGER", "---=== " +mHistoryManager.getRecords());
			mHistoryManager.getFirstRecord();
			break;
		case R.id.action_clear:
			mHistoryManager.clear();
			break;
		case R.id.action_delete_all:
			mHistoryManager.deleteAllRecords();
			break;
		}
		return true;
	}

	@Override
	protected int getLoggerProfileTitle() {
		return R.string.history_feature_title;
	}

	@Override
	protected int getAboutTextId() {
		return R.string.history_about_text;
	}

	@Override
	protected int getDefaultDeviceName() {
		return R.string.history_default_name;
	}

	@Override
	protected UUID getFilterUUID() {
		return HistoryManager.HIS_SERVICE_UUID;
	}

	@Override
	protected void setDefaultUI() {
		mHistoryManager.clear();
	}

	private void setOperationInProgress(final boolean progress) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				// setSupportProgressBarIndeterminateVisibility(progress);
				mControlPanelStd.setVisibility(!progress ? View.VISIBLE : View.GONE);
				mControlPanelAbort.setVisibility(progress ? View.VISIBLE : View.GONE);
			}
		});
	}

	@Override
	public void onDeviceDisconnected() {
		super.onDeviceDisconnected();
		setOperationInProgress(false);
	}

	@Override
	public void onOperationStarted() {
		setOperationInProgress(true);
	}

	@Override
	public void onOperationCompleted() {
		setOperationInProgress(false);
	}

	@Override
	public void onOperationAborted() {
		setOperationInProgress(false);
	}

	@Override
	public void onOperationNotSupported() {
		setOperationInProgress(false);
		showToast(R.string.history_operation_not_supported);
	}

	@Override
	public void onOperationFailed() {
		setOperationInProgress(false);
		showToast(R.string.history_operation_failed);
	}

	@Override
	public void onError(final String message, final int errorCode) {
		super.onError(message, errorCode);
		onOperationFailed();
	}

	@Override
	public void onDatasetChanged() {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				final SparseArray<HistoryRecord> records = mHistoryManager.getRecords();
				Log.i("mHistoryManager mHistoryManager MANGER  11", "---=== " +records);
				if (records.size() > 0) {
					final int unit = records.valueAt(0).unit;
					mUnitView.setVisibility(View.VISIBLE);
					mUnitView.setText(unit == HistoryRecord.UNIT_kgpl ? R.string.history_unit_mgpdl : R.string.history_unit_mmolpl);
				} else
					mUnitView.setVisibility(View.GONE);

			mAdapter.notifyDataSetChanged();
			}
		});
	}

	@Override
	public void onNumberOfRecordsRequested(final int value) {
		showToast(getString(R.string.history_progress, value));
	}
}
