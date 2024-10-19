#include <tuple>

namespace systems {
	struct particle_trait {
		template <typename ...Attributes>
		using particle_deconstruct = std::tuple<Attributes...>;
	};
}