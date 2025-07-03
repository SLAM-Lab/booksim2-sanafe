import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.preprocessing import StandardScaler
import warnings
warnings.filterwarnings('ignore')

def exploratory_analysis(df):
        """Comprehensive exploratory analysis of prediction errors"""
        print("=== EXPLORATORY ANALYSIS ===\n")

        # 1. Error distribution by key characteristics
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))

        # Error vs Hops
        axes[0,0].scatter(df['hops'], df['network_delay_error'], alpha=0.6)
        axes[0,0].set_xlabel('Number of Hops')
        axes[0,0].set_ylabel('Network Delay Error (s)')
        axes[0,0].set_title('Error vs Number of Hops')

        # Error vs Original Delay
        axes[0,1].scatter(df['network_delay'], df['network_delay_error'], alpha=0.6)
        axes[0,1].set_xlabel('Original Network Delay (s)')
        axes[0,1].set_ylabel('Network Delay Error (s)')
        axes[0,1].set_title('Error vs Original Delay')

        # Error vs Message Size (spikes)
        axes[0,2].scatter(df['spikes'], df['network_delay_error'], alpha=0.6)
        axes[0,2].set_xlabel('Number of Spikes')
        axes[0,2].set_ylabel('Network Delay Error (s)')
        axes[0,2].set_title('Error vs Message Size')

        # Percentage error distributions
        axes[1,0].hist(df['network_delay_pct_error'], bins=50, alpha=0.7)
        axes[1,0].set_xlabel('Percentage Error (%)')
        axes[1,0].set_ylabel('Frequency')
        axes[1,0].set_title('Distribution of Percentage Errors')

        # Error by hardware source/destination
        if 'src_hw' in df.columns:
            error_by_src = df.groupby('src_hw')['network_delay_pct_error'].mean()
            axes[1,1].bar(range(len(error_by_src)), error_by_src.values)
            axes[1,1].set_xlabel('Source Hardware')
            axes[1,1].set_ylabel('Mean % Error')
            axes[1,1].set_title('Error by Source Hardware')
            axes[1,1].set_xticks(range(len(error_by_src)))
            axes[1,1].set_xticklabels([f'{x:.1f}' for x in error_by_src.index], rotation=45)

        # Error vs Generation Delay (timing relationship)
        axes[1,2].scatter(df['generation_delay'], df['network_delay_error'], alpha=0.6)
        axes[1,2].set_xlabel('Generation Delay (s)')
        axes[1,2].set_ylabel('Network Delay Error (s)')
        axes[1,2].set_title('Error vs Generation Delay')

        plt.tight_layout()
        plt.savefig("figures/analysis.pdf")

        # 2. Statistical summary by categories
        print("=== ERROR PATTERNS BY CHARACTERISTICS ===")

        # By number of hops
        hop_analysis = df.groupby('hops').agg({
            'network_delay_pct_error': ['mean', 'std', 'count'],
            'network_delay_error': ['mean', 'std']
        }).round(4)
        print("\nError by Number of Hops:")
        print(hop_analysis)

        # By message size (spikes) bins
        df['spike_bins'] = pd.cut(df['spikes'], bins=5, labels=['Small', 'Med-Small', 'Medium', 'Med-Large', 'Large'])
        spike_analysis = df.groupby('spike_bins').agg({
            'network_delay_pct_error': ['mean', 'std', 'count'],
            'network_delay_error': ['mean', 'std']
        }).round(4)
        print("\nError by Message Size:")
        print(spike_analysis)


import numpy as np
def feature_engineering(df):
    """Create features for ML analysis"""
    target_cols = ['network_delay_error', 'network_delay', 'blocking_delay']

    # Basic features
    features_df = df.copy()

    # 1. Network topology features
    features_df['hops_squared'] = features_df['hops'] ** 2
    features_df['hops_log'] = np.log1p(features_df['hops'])

    # 2. Message characteristics
    features_df['spikes_log'] = np.log1p(features_df['spikes'])
    features_df['spikes_per_hop'] = features_df['spikes'] / (features_df['hops'] + 1)

    # 3. Timing relationships
    features_df['total_original_delay'] = (features_df['generation_delay'] +
                                            features_df['network_delay'] +
                                            features_df['processing_delay'])

    features_df['buffer_processing_product'] = (features_df['mean_process_cycles'] *
                                            features_df['buffered_path'])
    features_df['dest_buffer_delay_product'] = (features_df['mean_process_dest_cycles'] *
                                            features_df['buffered_dest'])

    # 5. Derived timing features
    feature_columns = [
        'hops',
        'spikes',
        'generation_delay',
        'processing_delay',
        'buffered_path',
        'mean_process_cycles',
        #'max_buffered',
        #'buffered_squared',
        'var_process_cycles',
        'sharing_along_flow',
        'mean_process_flow_cycles',
        'var_process_flow_cycles',
        'mean_process_path_cycles',
        'var_process_path_cycles',
        'sharing_along_path',
        'buffered_dest',
        'mean_process_dest_cycles',
        'full_dest_tile',
        'subnet',
        #'buffer_processing_product',
        # 'dest_buffer_delay_product'
    ]

    return features_df[feature_columns + target_cols].dropna(), feature_columns


def train_ml_models(features_df, feature_columns, original_data):
    """Train ML models to understand error patterns"""
    print("=== MACHINE LEARNING ANALYSIS ===\n")
    target_cols = ['network_delay_cycle',]

    X = features_df[feature_columns]

    results = {}
    for target in target_cols:
        print(f"\n--- Predicting {target} ---")
        y = original_data[target]

        # Split data
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2,
                                                            random_state=42)

        # Scale features
        feature_scaler = StandardScaler()
        X_train_scaled = feature_scaler.fit_transform(X_train)
        X_test_scaled = feature_scaler.transform(X_test)

        target_scaler = StandardScaler()
        y_train_scaled = target_scaler.fit_transform(y_train.values.reshape(-1, 1))

        # Train Random Forest
        rf = RandomForestRegressor(n_estimators=100, random_state=42, n_jobs=-1)
        rf.fit(X_train_scaled, y_train_scaled)

        # Predictions
        y_pred_scaled = rf.predict(X_test_scaled)
        y_pred = target_scaler.inverse_transform(y_pred_scaled.reshape(-1, 1)).flatten()
        print(y_pred.min())
        print(y_pred.max())
        input()

        # Metrics
        r2 = r2_score(y_test, y_pred)
        rmse = np.sqrt(mean_squared_error(y_test, y_pred))

        print(f"R² Score: {r2:.4f}")
        print(f"RMSE: {rmse:.6f}")

        # Feature importance
        feature_importance = pd.DataFrame({
            'feature': feature_columns,
            'importance': rf.feature_importances_
        }).sort_values('importance', ascending=False)

        print(f"\nTop 10 Most Important Features for {target}:")
        print(feature_importance.head(10))

        results[target] = {
            'model': rf,
            'feature_scaler': feature_scaler,
            'target_scaler': target_scaler,
            'feature_importance': feature_importance,
            'r2': r2,
            'rmse': rmse
        }

        # Plot feature importance
        plt.figure(figsize=(10, 6))
        top_features = feature_importance.head(15)
        plt.barh(range(len(top_features)), top_features['importance'])
        plt.yticks(range(len(top_features)), top_features['feature'])
        plt.xlabel('Feature Importance')
        plt.title(f'Feature Importance for {target}')
        plt.gca().invert_yaxis()
        plt.tight_layout()
        plt.savefig("figures/ml_features.pdf")

        # Scale all features and predict so we can compare predictor against
        #  timeseries etc
        X_all_scaled = feature_scaler.transform(X)
        y_pred_all_scaled = rf.predict(X_all_scaled)
        y_pred_all = target_scaler.inverse_transform(y_pred_all_scaled.reshape(-1, 1)).flatten()
        original_data['ml_predicted'] = y_pred_all

    return results

from sklearn.neural_network import MLPRegressor
def compare_ml_vs_sana_fe(features_df, feature_columns, original_df):
        """Compare ML approach vs SANA-FE for predicting cycle-accurate delays"""
        print("=== ML vs SANA-FE COMPARISON ===\n")

        X = features_df[feature_columns]
        # Target: Actual cycle-accurate delays (converted to seconds)
        y_network = original_df['network_delay_cycle']  # Ground truth from detailed simulator
        y_blocking = original_df['blocked_delay_cycle']  # Ground truth from detailed simulator

        results = {}
        for target_name, y, filename in [('Network Delay', y_network, "ml_net.pdf"), ('Blocking Delay', y_blocking, "ml_blocking.pdf")]:
            print(f"\n--- Predicting {target_name} (Cycle-Accurate) ---")

            # Split data
            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
            feature_scaler = StandardScaler()
            X_train_scaled = feature_scaler.fit_transform(X_train)
            X_test_scaled = feature_scaler.transform(X_test)

            target_scaler = StandardScaler()
            y_train_scaled = target_scaler.fit_transform(y_train.values.reshape(-1, 1))

            # # Scale features for ML
            # feature_scaler = StandardScaler()
            # X_train_scaled = feature_scaler.fit_transform(X_train)
            # X_test_scaled = feature_scaler.transform(X_test)

            # target_scaler = StandardScaler()
            # y_train_scaled = target_scaler.fit_transform(y_train.values.reshape(-1, 1))
            # y_test_scaled = target_scaler.transform(y_test.values.reshape(-1, 1))

            # Configure MLP
            # rf = MLPRegressor(
            #     hidden_layer_sizes=(128, 64, 32),  # 3 hidden layers
            #     activation='relu',                  # ReLU activation
            #     solver='adam',                     # Adam optimizer
            #     alpha=0.001,                       # L2 regularization
            #     learning_rate_init=0.001,          # Learning rate
            #     max_iter=500,                      # Max epochs
            #     early_stopping=True,               # Stop when validation doesn't improve
            #     validation_fraction=0.1,           # Use 10% of training for validation
            #     n_iter_no_change=20,               # Patience for early stopping
            #     random_state=42
            # )

            # Train ML model
            rf = RandomForestRegressor(n_estimators=100, random_state=42, n_jobs=-1)
            #rf = LinearRegression()

            rf.fit(X_train_scaled, y_train_scaled)
            ml_pred_scaled = rf.predict(X_test_scaled)
            ml_pred = target_scaler.inverse_transform(ml_pred_scaled.reshape(-1, 1)).flatten()

            print(ml_pred.min())
            print(ml_pred.max())
            input()

            # Get SANA-FE predictions for same test set
            test_indices = X_test.index
            if target_name == 'Network Delay':
                sana_pred = original_df.loc[test_indices, 'network_delay']
            else:  # Blocking Delay
                sana_pred = original_df.loc[test_indices, 'blocking_delay']

            # Calculate metrics for both
            ml_r2 = r2_score(y_test, ml_pred)
            ml_rmse = np.sqrt(mean_squared_error(y_test, ml_pred))
            ml_mae = np.mean(np.abs((y_test - ml_pred)))

            sana_r2 = r2_score(y_test, sana_pred)
            sana_rmse = np.sqrt(mean_squared_error(y_test, sana_pred))
            sana_mae = np.mean(np.abs((y_test - sana_pred)))


            # Calculate percentage improvements
            r2_improvement = ((ml_r2 - sana_r2) / abs(sana_r2)) * 100 if sana_r2 != 0 else float('inf')
            rmse_improvement = ((sana_rmse - ml_rmse) / sana_rmse) * 100

            print(f"**ML Model Performance:**")
            print(f"  R² Score: {ml_r2:.4f}")
            print(f"  RMSE: {ml_rmse:.6e}")
            print(f"  MAE: {ml_mae:.6e}")

            print(f"**SANA-FE Performance:**")
            print(f"  R² Score: {sana_r2:.4f}")
            print(f"  RMSE: {sana_rmse:.6e}")
            print(f"  MAE: {sana_mae:.6e}")

            print(f"**Improvement:**")
            print(f"  R² improvement: {r2_improvement:+.1f}%")
            print(f"  RMSE improvement: {rmse_improvement:+.1f}%")

            # Store results
            results[target_name] = {
                'ml_r2': ml_r2, 'ml_rmse': ml_rmse,
                'sana_r2': sana_r2, 'sana_rmse': sana_rmse,
                'r2_improvement': r2_improvement,
                'rmse_improvement': rmse_improvement,
                'model': rf,
                'target_scaler': target_scaler,
                'feature_scaler': feature_scaler
            }

            # Create comparison plot - both models on same plot
            plt.figure(figsize=(10, 5))

            # Subplot 1: Both models together
            plt.subplot(1, 2, 1)
            plt.scatter(ml_pred, y_test, alpha=0.4, label=f'ML (R² = {ml_r2:.3f})', color='blue', s=30)
            #plt.scatter(sana_pred, y_test, alpha=0.4, label=f'SANA-FE (R² = {sana_r2:.3f})', color='orange', s=30)

            # Perfect prediction line (y=x)
            all_values = np.concatenate([y_test, ml_pred, sana_pred])
            min_val, max_val = all_values.min(), all_values.max()
            plt.plot([min_val, max_val], [min_val, max_val], 'r--', lw=2, label='Perfect Prediction (y=x)')

            plt.xlabel('Predicted')
            plt.ylabel('Ground Truth (Cycle-Accurate)')
            plt.title(f'{target_name}: ML vs SANA-FE')
            plt.legend()
            plt.grid(True, alpha=0.3)

            # Subplot 2: Error distribution comparison
            plt.subplot(1, 2, 2)
            ml_errors = np.abs(y_test - ml_pred)
            sana_errors = np.abs(y_test - sana_pred)

            plt.hist(ml_errors, bins=30, alpha=0.6, label=f'ML Errors (RMSE={ml_rmse:.2e})', color='blue')
            plt.hist(sana_errors, bins=30, alpha=0.6, label=f'SANA-FE Errors (RMSE={sana_rmse:.2e})', color='orange')
            plt.xlabel('Absolute Error')
            plt.ylabel('Frequency')
            plt.title(f'{target_name}: Error Distribution')
            plt.legend()
            plt.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.show()
            plt.savefig(f"figures/{filename}")

        return results

def merge_message_dataframes(messages_csv, detailed_csv, cycle_period=1.0e-9):
    """
    Merge two CSV dataframes: messages data and detailed simulator data.

    Parameters:
    - messages_csv: path to the messages CSV file
    - detailed_csv: path to the detailed simulator CSV file
    - cycle_period: cycle period in seconds (default: 1.0e-9 ns)

    Returns:
    - merged_df: combined dataframe
    """

    # Read the CSV files
    messages_df = pd.read_csv(messages_csv)
    messages_df = messages_df[messages_df['mid'] >= 0]
    print(f"Total messages in sana-fe: {len(messages_df)}")
    detailed_df = pd.read_csv(detailed_csv)
    print(f"Total messages in sana-fe: {len(detailed_df)}")

    # Convert cycle-based delays to seconds in the detailed dataframe
    detailed_df['blocked_delay_cycle'] = detailed_df['blocked_cycles'] * cycle_period
    detailed_df['network_delay_cycle'] = detailed_df['network_cycles'] * cycle_period

    # Merge the dataframes on 'mid' (message ID)
    # Using left join to keep all messages from the first dataframe
    merged_df = pd.merge(messages_df, detailed_df, on='mid', how='left')

    # Compare network delays between the two datasets
    merged_df['network_delay_error'] = merged_df['network_delay_cycle'] - merged_df['network_delay']
    merged_df['blocking_delay_error'] = merged_df['blocked_delay_cycle'] - merged_df['blocking_delay']

    # Calculate percentage errors (avoiding division by zero)
    merged_df['network_delay_pct_error'] = (merged_df['network_delay_error'] /
                                          merged_df['network_delay'].replace(0, float('nan'))) * 100
    merged_df['blocking_delay_pct_error'] = (merged_df['blocking_delay_error'] /
                                           merged_df['blocking_delay'].replace(0, float('nan'))) * 100

    return merged_df

def analyze_differences(merged_df):
    """
    Analyze the differences between the two datasets.
    """
    print("=== Merge Summary ===")
    print(f"Total messages in dataset: {len(merged_df)}")

    # Analyze differences for matched messages
    matched_df = merged_df.dropna(subset=['blocking_delay'])

    if len(matched_df) > 0:
        print("\n=== Network Delay Comparison (for matched messages) ===")
        print(f"Mean (SANA-FE): {matched_df['network_delay'].mean():.2e} seconds")
        print(f"Mean (Booksim 2): {matched_df['network_delay_cycle'].mean():.2e} seconds")
        print(f"Mean difference: {matched_df['network_delay_error'].mean():.2e} seconds")
        print(f"Std difference: {matched_df['network_delay_error'].std():.2e} seconds")
        print(f"Max absolute difference: {matched_df['network_delay_error'].abs().max():.2e} seconds")
        print(f"Mean percentage error: {matched_df['network_delay_pct_error'].mean():.2f}%")
        print(f"Std percentage error: {matched_df['network_delay_pct_error'].std():.2f}%")
        print(f"Max absolute percentage error: {matched_df['network_delay_pct_error'].abs().max():.2f}%")

        print("\n=== Blocking Delay Comparison ===")
        print(f"Mean (SANA-FE): {matched_df['blocking_delay'].mean():.2e} seconds")
        print(f"Mean (Booksim 2): {matched_df['blocked_delay_cycle'].mean():.2e} seconds")
        print(f"Mean difference: {matched_df['blocking_delay_error'].mean():.2e} seconds")
        print(f"Std difference: {matched_df['blocking_delay_error'].std():.2e} seconds")
        print(f"Max absolute difference: {matched_df['blocking_delay_error'].abs().max():.2e} seconds")
        print(f"Mean percentage error: {matched_df['blocking_delay_pct_error'].mean():.2f}%")
        print(f"Std percentage error: {matched_df['blocking_delay_pct_error'].std():.2f}%")
        print(f"Max absolute percentage error: {matched_df['blocking_delay_pct_error'].abs().max():.2f}%")

        plt.figure(figsize=(4, 4))
        plt.plot(merged_df['network_delay'].to_numpy() * 1.0e6,
                 merged_df['network_delay_cycle'].to_numpy() * 1.0e6, "o", alpha=0.5)
        plt.xlabel("SANA-FE [us]")
        plt.ylabel("Booksim2 [us]")
        plt.savefig("figures/network_delay.pdf")

        plt.figure(figsize=(4, 4))
        plt.plot(merged_df['blocking_delay'].to_numpy() * 1.0e6,
                 merged_df['blocked_delay_cycle'].to_numpy() * 1.0e6, "o", alpha=0.5)
        plt.xlabel("SANA-FE [us]")
        plt.ylabel("Booksim2 [us]")
        plt.savefig("figures/blocking_delay.pdf")

        plt.figure(figsize=(4, 4))
        plt.plot(merged_df['mid'].to_numpy(),
                 merged_df['network_delay_error'].to_numpy() * 1.0e6, "o", alpha=0.3, markersize=0.7)
        plt.xlabel("Message ID")
        plt.ylabel("Network Delay Error [us]")
        plt.savefig("figures/network_error.pdf")

        plt.figure(figsize=(4, 4))
        plt.plot(merged_df['mid'].to_numpy(),
                 merged_df['blocking_delay_error'].to_numpy() * 1.0e6, "o", alpha=0.3, markersize=0.7)
        plt.xlabel("Message ID")
        plt.ylabel("Blocking Delay Error [us]")
        plt.savefig("figures/blocking_error.pdf")

# Example usage
if __name__ == "__main__":
    # Replace with your actual file paths
    messages_file = "messages_single_ts.csv"
    detailed_file = "messages_cycle_accurate.csv"

    # Merge the dataframes
    merged_data = merge_message_dataframes(messages_file, detailed_file)

    # Display basic info
    print("Merged dataframe shape:", merged_data.shape)
    print("\nFirst few rows:")
    print(merged_data.head())

    # Analyze differences
    analyze_differences(merged_data)

    # Optional: Display columns for reference
    print("\n=== Column Information ===")
    print("Original columns:", list(merged_data.columns[:15]))  # First 15 columns
    print("Added columns:", ['blocked_delay', 'network_delay', 'blocked_delay_sec',
                           'network_delay_sec', 'network_delay_error', 'blocking_delay_error'])

    features_df, feature_cols = feature_engineering(merged_data)
    train_ml_models(features_df, feature_cols, merged_data)
    compare_ml_vs_sana_fe(features_df, feature_cols, merged_data)

    # Save the merged dataframe with predictions
    merged_data.to_csv("merged_messages.csv", index=False)
    print("\nMerged data saved to 'merged_messages.csv'")