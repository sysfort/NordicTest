
package nordic.android.nrf;

import java.util.Calendar;

public class HistoryRecord {
	public static final int UNIT_kgpl = 0;
	public static final int UNIT_molpl = 1;

	
	protected int sequenceNumber;
	
	protected Calendar time;
	
	protected int timeOffset;

	protected float glucoseConcentration;
	
	protected int unit;
	
	protected int type;
	
	protected int sampleLocation;

	protected int status;

	protected MeasurementContext context;

	public static class MeasurementContext {
		public static final int UNIT_kg = 0;
		public static final int UNIT_l = 1;

		
		protected int carbohydrateId;
		
		protected float carbohydrateUnits;
		
		protected int meal;
		
		protected int tester;
		
		protected int health;
		
		protected int exerciseDuration;
	
		protected int exerciseIntensity;
		
		protected int medicationId;
	
		protected float medicationQuantity;
		
		protected int medicationUnit;
		
		protected float HbA1c;
	}
}
