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
    avg_reward: float


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
            avg_reward=data.get("avg_reward", 0.0),
            batch_size=data.get("batch_size", 0),
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
            "avg_reward",
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
                    avg_reward=float(values[6]),
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
                plt.subplot(int(ceil(num_plots / 2)), 2, i + 1)
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
    data = DataLogger.load_from_csv(
        "export/kris_robot1_training_6a8fec3e-4b6d-4384-8288-87c666d0ecff.csv"
    )
    DataLogger.subplot_data(
        data, ["loss", "policy_loss", "avg_reward", "entropy"], "export/subplot.png", "Batch"
    )
