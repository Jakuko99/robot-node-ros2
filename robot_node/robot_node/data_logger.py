import matplotlib.pyplot as plt
from dataclasses import dataclass


@dataclass
class Batch:
    batch_nr: int
    batch_size: int
    loss: float
    actor_loss: float
    critic_loss: float
    alpha_loss: float
    entropy: float
    alpha: float
    avg_reward: float
    updates: float


class DataLogger:
    def __init__(self, log_file):
        self.log_file = log_file
        self.data: dict[int, Batch] = {}
        self.batch_nr: int = 0

    def log_batch(self, data: dict[str, float]):
        self.data[self.batch_nr] = Batch(
            batch_nr=self.batch_nr,
            loss=data.get("loss", 0.0),
            actor_loss=data.get("actor_loss", 0.0),
            critic_loss=data.get("critic_loss", 0.0),
            alpha_loss=data.get("alpha_loss", 0.0),
            entropy=data.get("entropy", 0.0),
            alpha=data.get("alpha", 0.0),
            avg_reward=data.get("avg_reward", 0.0),
            updates=data.get("updates", 0),
            batch_size=data.get("batch_size", 0),
        )
        self.batch_nr += 1

    def export_to_csv(self):
        with open(self.log_file, mode="w", newline="") as f:
            fieldnames = [
                "batch_nr",
                "batch_size",
                "loss",
                "actor_loss",
                "critic_loss",
                "alpha_loss",
                "entropy",
                "alpha",
                "avg_reward",
                "updates",
            ]
            with open(self.log_file, "w") as f:
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
                values: list = line.strip().split(",")
                batch = Batch(
                    batch_nr=int(values[0]),
                    batch_size=int(values[1]),
                    loss=float(values[2]),
                    actor_loss=float(values[3]),
                    critic_loss=float(values[4]),
                    alpha_loss=float(values[5]),
                    entropy=float(values[6]),
                    alpha=float(values[7]),
                    avg_reward=float(values[8]),
                    updates=float(values[9]),
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


if __name__ == "__main__":
    data = DataLogger.load_from_csv(
        "export/kris_robot1_training_a8b860a1-de7b-4790-96fb-0222fe2f9ec0.csv"
    )
    DataLogger.plot_data(data, "loss", "export/loss_plot.png")
    DataLogger.plot_data(data, "avg_reward", "export/avg_reward_plot.png")
