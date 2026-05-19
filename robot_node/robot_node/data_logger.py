import matplotlib.pyplot as plt
from dataclasses import dataclass
from math import ceil


@dataclass
class Batch:
    batch_nr: int
    batch_size: int
    loss: float
    policy_loss: float
    value_loss: float
    entropy: float
    entropy_coef: float
    coverage_gain: float = 0.0
    frontier_gain: float = 0.0
    overlap_growth: float = 0.0
    crowding_penalty: float = 0.0
    redundancy_penalty: float = 0.0
    nearest_teammate: float = 0.0
    team_context_mean: float = 0.0
    replay_size: int = 0
    training_steps: int = 0
    off_policy_updates: int = 0
    avg_reward: float = 0.0


class DataLogger:
    def __init__(self):
        self.data: dict[int, Batch] = {}
        self.batch_nr: int = 0

    def log_batch(self, data: dict[str, float]):
        self.data[self.batch_nr] = Batch(
            batch_nr=self.batch_nr,
            loss=data.get("loss", 0.0),
            policy_loss=data.get("policy_loss", 0.0),
            value_loss=data.get("value_loss", 0.0),
            entropy=data.get("entropy", 0.0),
            entropy_coef=data.get("entropy_coef", 0.0),
            coverage_gain=data.get("coverage_gain", 0.0),
            frontier_gain=data.get("frontier_gain", 0.0),
            overlap_growth=data.get("overlap_growth", 0.0),
            crowding_penalty=data.get("crowding_penalty", 0.0),
            redundancy_penalty=data.get("redundancy_penalty", 0.0),
            nearest_teammate=data.get("nearest_teammate", 0.0),
            team_context_mean=data.get("team_context_mean", 0.0),
            avg_reward=data.get("avg_reward", 0.0),
            batch_size=data.get("batch_size", 0),
            replay_size=data.get("replay_size", 0),
            training_steps=data.get("training_steps", 0),
            off_policy_updates=data.get("off_policy_updates", 0),
        )
        self.batch_nr += 1

    def export_to_csv(self, log_file: str):
        fieldnames: list[str] = [
            "batch_nr",
            "batch_size",
            "loss",
            "policy_loss",
            "value_loss",
            "entropy",
            "entropy_coef",
            "coverage_gain",
            "frontier_gain",
            "overlap_growth",
            "crowding_penalty",
            "redundancy_penalty",
            "nearest_teammate",
            "team_context_mean",
            "avg_reward",
            "replay_size",
            "training_steps",
            "off_policy_updates",
        ]

        with open(log_file, "w") as f:
            f.write(",".join(fieldnames) + "\n")
            for batch in self.data.values():
                f.write(",".join(str(getattr(batch, field)) for field in fieldnames) + "\n")

    @staticmethod
    def load_from_csv(csv_file: str) -> dict[int, Batch]:
        data: dict[int, Batch] = {}
        with open(csv_file, mode="r") as f:
            for line in f.readlines():
                if line.startswith("batch_nr"):
                    continue  # skip header
                values: list[str] = line.strip().split(",")
                batch = Batch(
                    batch_nr=int(values[0]),
                    batch_size=int(values[1]),
                    loss=float(values[2]),
                    policy_loss=float(values[3]),
                    value_loss=float(values[4]),
                    entropy=float(values[5]),
                    entropy_coef=float(values[6]),
                    coverage_gain=float(values[7]) if len(values) > 7 else 0.0,
                    frontier_gain=float(values[8]) if len(values) > 8 else 0.0,
                    overlap_growth=float(values[9]) if len(values) > 9 else 0.0,
                    crowding_penalty=float(values[10]) if len(values) > 10 else 0.0,
                    redundancy_penalty=float(values[11]) if len(values) > 11 else 0.0,
                    nearest_teammate=float(values[12]) if len(values) > 12 else 0.0,
                    team_context_mean=float(values[13]) if len(values) > 13 else 0.0,
                    avg_reward=float(values[14]) if len(values) > 14 else 0.0,
                    replay_size=int(values[15]) if len(values) > 15 else 0,
                    training_steps=int(values[16]) if len(values) > 16 else 0,
                    off_policy_updates=int(values[17]) if len(values) > 17 else 0,
                )
                data[batch.batch_nr] = batch
        return data

    @staticmethod
    def plot_data(data: dict[int, Batch], plot_label: str, save_path: str):
        batch_nrs = [batch.batch_nr for batch in data.values()]

        if plot_label in Batch.__dataclass_fields__:
            values = [getattr(batch, plot_label) for batch in data.values()]
            plt.figure(figsize=(10, 5))
            plt.plot(batch_nrs, values, label=plot_label)
            plt.xlabel("Batch Number")
            plt.ylabel(plot_label.replace("_", " ").title())
            plt.title(f"{plot_label.replace('_', ' ').title()} over Batches")
            plt.legend()
            plt.grid()
            plt.savefig(save_path)

        else:
            print(
                f"Error: Invalid plot label '{plot_label}'. Valid labels are: {list(Batch.__dataclass_fields__.keys())}"
            )

    @staticmethod
    def subplot_data(
        data: dict[int, Batch], plot_labels: list[str], save_path: str, data_label: str = "Batch"
    ):
        batch_nrs = [batch.batch_nr for batch in data.values()]
        num_plots = len(plot_labels)
        plt.figure(figsize=(15, 5 * int(ceil(num_plots / 2))))

        for i, plot_label in enumerate(plot_labels):
            if plot_label in Batch.__dataclass_fields__:
                values = [getattr(batch, plot_label) for batch in data.values()]
                plt.subplot(int(ceil(num_plots / 2)), 3, i + 1)
                plt.plot(batch_nrs, values, label=plot_label)
                plt.xlabel(data_label)
                plt.ylabel(plot_label.replace("_", " ").title())
                plt.title(f"{plot_label.replace('_', ' ').title()} over {data_label}s")
                # plt.legend()
                plt.grid()
            else:
                print(
                    f"Error: Invalid plot label '{plot_label}'. Valid labels are: {list(Batch.__dataclass_fields__.keys())}"
                )

        plt.tight_layout()
        plt.savefig(save_path)


if __name__ == "__main__":
    data = DataLogger.load_from_csv("export/kris_robot1_training_1.csv")
    DataLogger.subplot_data(
        data, ["loss", "policy_loss", "avg_reward", "entropy"], "export/subplot.png", "Batch"
    )
