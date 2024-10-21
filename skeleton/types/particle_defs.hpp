#include <tuple>
#include <type_traits>

namespace systems {
	struct particle_trait {

		template <typename ...input_t>
		using tuple_cat_t = decltype(std::tuple_cat(std::declval<input_t>()...));

		template<typename... Ts> struct is_tuple : std::false_type {};
		template<typename... Ts> struct is_tuple<std::tuple<Ts...>> : std::true_type {};
		template<typename... Ts> struct is_tuple<const std::tuple<Ts...>> : std::true_type {};
		template<typename... Ts> struct is_tuple<volatile std::tuple<Ts...>> : std::true_type {};
		template<typename... Ts> struct is_tuple<const volatile std::tuple<Ts...>> : std::true_type {};

		template <
			typename Tuple,
			typename ...Attributes
		>
		using particle_deconstruct = tuple_cat_t<
			std::conditional_t<
				is_tuple<Tuple>::value,
				Tuple,
				std::tuple<Tuple>
			>,
			std::tuple<Attributes...>
		>;
	};
}