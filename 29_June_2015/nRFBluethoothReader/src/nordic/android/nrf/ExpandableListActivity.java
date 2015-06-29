/*
 * Copyright (C) 2006 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package nordic.android.nrf;

import android.app.Activity;
import android.database.Cursor;
import android.os.Bundle;
import android.support.v7.app.ActionBarActivity;
import android.util.Log;
import android.view.ContextMenu;
import android.view.ContextMenu.ContextMenuInfo;
import android.view.View;
import android.view.View.OnCreateContextMenuListener;
import android.widget.ExpandableListAdapter;
import android.widget.ExpandableListView;
import android.widget.SimpleCursorTreeAdapter;
import android.widget.SimpleExpandableListAdapter;

import java.util.List;
import java.util.Map;

import nordic.android.nrf.R;


public class ExpandableListActivity extends ActionBarActivity implements
		OnCreateContextMenuListener,
		ExpandableListView.OnChildClickListener, ExpandableListView.OnGroupCollapseListener,
		ExpandableListView.OnGroupExpandListener {
	ExpandableListAdapter mAdapter;
	ExpandableListView mList;
	boolean mFinishedStart = false;

	/**
	 * Override this to populate the context menu when an item is long pressed. menuInfo will contain an
	 * {@link android.widget.ExpandableListView.ExpandableListContextMenuInfo} whose packedPosition is a packed position that should be used with
	 * {@link ExpandableListView#getPackedPositionType(long)} and the other similar methods.
	 * <p>
	 * {@inheritDoc}
	 */
	@Override
	public void onCreateContextMenu(ContextMenu menu, View v, ContextMenuInfo menuInfo) {
	}

	/**
	 * Override this for receiving callbacks when a child has been clicked.
	 * <p>
	 * {@inheritDoc}
	 */
	@Override
	public boolean onChildClick(ExpandableListView parent, View v, int groupPosition,
			int childPosition, long id) {
		return false;
	}

	/**
	 * Override this for receiving callbacks when a group has been collapsed.
	 */
	@Override
	public void onGroupCollapse(int groupPosition) {
	}

	/**
	 * Override this for receiving callbacks when a group has been expanded.
	 */
	@Override
	public void onGroupExpand(int groupPosition) {
	}

	/**
	 * Ensures the expandable list view has been created before Activity restores all of the view states.
	 * 
	 * @see Activity#onRestoreInstanceState(Bundle)
	 */
	@Override
	protected void onRestoreInstanceState(Bundle state) {
		ensureList();
		super.onRestoreInstanceState(state);
	}

	/**
	 * Updates the screen state (current list and other views) when the content changes.
	 * 
	 * @see ActionBarActivity#onSupportContentChanged()
	 */
	@Override
	public void onSupportContentChanged() {
		//super.onContentChanged();
		View emptyView = findViewById(R.id.empty);
		mList = (ExpandableListView) findViewById(R.id.list);
		if (mList == null) {
			throw new RuntimeException(
					"Your content must have a ExpandableListView whose id attribute is " +
							"'R.id.list'");
		}
		if (emptyView != null) {
			mList.setEmptyView(emptyView);
		}
		mList.setOnChildClickListener(this);
		mList.setOnGroupExpandListener(this);
		mList.setOnGroupCollapseListener(this);

		if (mFinishedStart) {
			setListAdapter(mAdapter);
		}
		Log.i("mGlucoseManager MANGER list ", "---=== " );
		mFinishedStart = true;
	}

	/**
	 * Provide the adapter for the expandable list.
	 */
	public void setListAdapter(ExpandableListAdapter adapter) {
		synchronized (this) {
			ensureList();
			mAdapter = adapter;
			mList.setAdapter(adapter);
		}
	}

	/**
	 * Get the activity's expandable list view widget. This can be used to get the selection, set the selection, and many other useful functions.
	 * 
	 * @see ExpandableListView
	 */
	public ExpandableListView getExpandableListView() {
		ensureList();
		return mList;
	}

	/**
	 * Get the ExpandableListAdapter associated with this activity's ExpandableListView.
	 */
	public ExpandableListAdapter getExpandableListAdapter() {
		return mAdapter;
	}

	private void ensureList() {
		if (mList != null) {
			return;
		}
		setContentView(R.layout.expandable_list_content);
	}

	/**
	 * Gets the ID of the currently selected group or child.
	 * 
	 * @return The ID of the currently selected group or child.
	 */
	public long getSelectedId() {
		return mList.getSelectedId();
	}

	/**
	 * Gets the position (in packed position representation) of the currently selected group or child. Use {@link ExpandableListView#getPackedPositionType},
	 * {@link ExpandableListView#getPackedPositionGroup}, and {@link ExpandableListView#getPackedPositionChild} to unpack the returned packed position.
	 * 
	 * @return A packed position representation containing the currently selected group or child's position and type.
	 */
	public long getSelectedPosition() {
		return mList.getSelectedPosition();
	}

	/**
	 * Sets the selection to the specified child. If the child is in a collapsed group, the group will only be expanded and child subsequently selected if
	 * shouldExpandGroup is set to true, otherwise the method will return false.
	 * 
	 * @param groupPosition
	 *            The position of the group that contains the child.
	 * @param childPosition
	 *            The position of the child within the group.
	 * @param shouldExpandGroup
	 *            Whether the child's group should be expanded if it is collapsed.
	 * @return Whether the selection was successfully set on the child.
	 */
	public boolean setSelectedChild(int groupPosition, int childPosition, boolean shouldExpandGroup) {
		return mList.setSelectedChild(groupPosition, childPosition, shouldExpandGroup);
	}

	/**
	 * Sets the selection to the specified group.
	 * 
	 * @param groupPosition
	 *            The position of the group that should be selected.
	 */
	public void setSelectedGroup(int groupPosition) {
		mList.setSelectedGroup(groupPosition);
	}

}
